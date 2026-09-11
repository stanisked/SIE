"""One-command supervised executor for the bounded ESP32 motion API.

This module deliberately does not plan, retry, acknowledge faults, or start a
second command.  It turns one already validated bridge plan into one explicit
operator-supervised HTTP request and returns a JSON-safe execution record.
"""
from __future__ import annotations

import json
import math
import re
import time
from datetime import datetime, timezone
from typing import Any, Callable
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode
from urllib.request import Request, urlopen


SCHEMA_VERSION = "sie.supervised_bounded_executor.v1"
BOOT_SESSION_RE = re.compile(r"[0-9A-F]{16}\Z")
COMMAND_ID_RE = re.compile(r"[A-Za-z0-9._-]{1,64}\Z")
ALLOWED_ENDPOINTS = frozenset({"/move-forward", "/turn-left", "/turn-right"})
TERMINAL_COMMAND_STATES = frozenset({"SUCCESS", "PARTIAL_PROGRESS", "FAULT", "STOPPED"})


class ExecutionContractError(ValueError):
    """A local plan or authorization cannot be used for execution."""


HttpRequest = Callable[[str, str, float], tuple[int, dict[str, Any]]]


def _json_safe(value: object, name: str) -> Any:
    try:
        return json.loads(json.dumps(value, allow_nan=False, sort_keys=True))
    except (TypeError, ValueError) as error:
        raise ExecutionContractError(f"{name} must be JSON-safe") from error


def _text(value: object, name: str) -> str:
    if type(value) is not str or not value:
        raise ExecutionContractError(f"{name} must be a non-empty string")
    return value


def _finite(value: object, name: str) -> float:
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ExecutionContractError(f"{name} must be finite")
    return float(value)


def _bounded_parameter(value: object, name: str) -> float:
    """Accept the bridge's canonical decimal string without normalizing it."""
    if type(value) in (int, float):
        return _finite(value, name)
    if type(value) is not str or re.fullmatch(r"(?:0|[1-9][0-9]*)(?:\.[0-9]+)?", value) is None:
        raise ExecutionContractError(f"{name} must be a canonical positive decimal")
    number = float(value)
    if not math.isfinite(number):
        raise ExecutionContractError(f"{name} must be finite")
    return number


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _record(
    *,
    result: str,
    reason: str | None,
    planned_command: dict[str, Any] | None,
    authorization: dict[str, Any] | None,
    preflight_status: dict[str, Any] | None = None,
    request_response: dict[str, Any] | None = None,
    terminal_status: dict[str, Any] | None = None,
    network_performed: bool = False,
    motor_command_performed: bool = False,
    reobserve_required: bool = True,
) -> dict[str, Any]:
    return _json_safe(
        {
            "schema_version": SCHEMA_VERSION,
            "timestamp": _utc_now(),
            "result": result,
            "reason": reason,
            "planned_command": planned_command,
            "operator_authorization": authorization,
            "preflight_status": preflight_status,
            "request_response": request_response,
            "terminal_status": terminal_status,
            "network_performed": network_performed,
            "motor_command_performed": motor_command_performed,
            "reobserve_required": reobserve_required,
        },
        "execution record",
    )


def validate_planned_bounded_command(value: object) -> dict[str, Any]:
    """Validate the bridge's bounded plan without creating or correcting it."""
    command = _json_safe(value, "planned_command")
    if type(command) is not dict:
        raise ExecutionContractError("planned_command must be an object")
    if command.get("result") != "PLANNED_BOUNDED_COMMAND":
        raise ExecutionContractError("planned_command must be PLANNED_BOUNDED_COMMAND")
    if command.get("network_performed") is not False:
        raise ExecutionContractError("planned_command must declare network_performed=false")
    if command.get("method") != "POST":
        raise ExecutionContractError("planned_command method must be POST")
    endpoint = _text(command.get("endpoint"), "planned_command.endpoint")
    if endpoint not in ALLOWED_ENDPOINTS:
        raise ExecutionContractError("planned_command endpoint is not a bounded motion endpoint")
    query = command.get("query")
    if type(query) is not dict:
        raise ExecutionContractError("planned_command.query must be an object")
    session = _text(query.get("boot_session_id"), "planned_command.query.boot_session_id")
    command_id = _text(query.get("command_id"), "planned_command.query.command_id")
    if BOOT_SESSION_RE.fullmatch(session) is None:
        raise ExecutionContractError("planned_command boot_session_id is invalid")
    if COMMAND_ID_RE.fullmatch(command_id) is None:
        raise ExecutionContractError("planned_command command_id is invalid")
    parameter = "distance_m" if endpoint == "/move-forward" else "angle_deg"
    expected = {"boot_session_id", "command_id", parameter}
    if set(query) != expected:
        raise ExecutionContractError("planned_command query fields do not match endpoint")
    amount = _bounded_parameter(query[parameter], f"planned_command.query.{parameter}")
    lower, upper = ((0.02, 0.10) if parameter == "distance_m" else (1.0, 10.0))
    if amount < lower or amount > upper:
        raise ExecutionContractError("planned_command parameter is outside bounded API range")
    return command


def authorize_operator_trial(
    *,
    planned_command: object,
    confirmation_command_id: object,
    authorization_mode: object,
    experimental_reason: object | None = None,
) -> dict[str, Any]:
    """Create an explicit local authorization, never a capability mutation."""
    command = validate_planned_bounded_command(planned_command)
    command_id = command["query"]["command_id"]
    if confirmation_command_id != command_id:
        raise ExecutionContractError("operator confirmation command_id does not match plan")
    mode = _text(authorization_mode, "authorization_mode")
    if mode not in {"QUALIFIED", "SUPERVISED_EXPERIMENTAL_TRIAL"}:
        raise ExecutionContractError("unsupported authorization_mode")
    reason: str | None = None
    if mode == "SUPERVISED_EXPERIMENTAL_TRIAL":
        reason = _text(experimental_reason, "experimental_reason")
    elif experimental_reason is not None:
        raise ExecutionContractError("experimental_reason is only valid for supervised experimental trial")
    return _json_safe(
        {
            "mode": mode,
            "confirmed_command_id": command_id,
            "experimental_reason": reason,
            "automatic_capability_qualification_changed": False,
        },
        "operator authorization",
    )


def _http_request(method: str, url: str, timeout_s: float) -> tuple[int, dict[str, Any]]:
    request = Request(url, method=method, headers={"Accept": "application/json"})
    try:
        with urlopen(request, timeout=timeout_s) as response:  # nosec B310: explicit local operator endpoint
            status = int(response.status)
            body = response.read().decode("utf-8")
    except HTTPError as error:
        status = int(error.code)
        body = error.read().decode("utf-8", errors="replace")
    except (URLError, OSError) as error:
        raise ExecutionContractError(f"HTTP transport error: {error}") from error
    try:
        payload = json.loads(body)
    except json.JSONDecodeError as error:
        raise ExecutionContractError("HTTP response is not a JSON object") from error
    if type(payload) is not dict:
        raise ExecutionContractError("HTTP response must be a JSON object")
    return status, _json_safe(payload, "HTTP response")


def _ready_status(status: dict[str, Any], command: dict[str, Any]) -> str | None:
    if status.get("motion_api_version") != "v2_bounded_motion_api":
        return "UNEXPECTED_MOTION_API_VERSION"
    if status.get("boot_session_id") != command["query"]["boot_session_id"]:
        return "BOOT_SESSION_CHANGED_OR_PLAN_NOT_FRESH"
    if status.get("state") != "READY":
        return "CONTROLLER_NOT_READY"
    if status.get("bounded_fault_latched") is not False:
        return "BOUNDED_FAULT_LATCHED"
    if status.get("active_command_id") is not None:
        return "ACTIVE_COMMAND_PRESENT"
    return None


def _bounded_url(base_url: str, endpoint: str, query: dict[str, Any]) -> str:
    root = base_url.rstrip("/")
    if not root.startswith("http://"):
        raise ExecutionContractError("base_url must use explicit http://")
    return root + endpoint + "?" + urlencode(query)


def execute_one_supervised_command(
    *,
    planned_command: object,
    authorization: object,
    base_url: object,
    request: HttpRequest = _http_request,
    timeout_s: float = 2.0,
    poll_interval_s: float = 0.2,
    terminal_timeout_s: float = 8.0,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> dict[str, Any]:
    """Execute one bounded command once, then await a terminal status.

    No POST is retried. A fault is only recorded; this function never calls
    `/ack-fault`, `/stop`, or a corrective movement endpoint.
    """
    command: dict[str, Any] | None = None
    auth: dict[str, Any] | None = None
    try:
        command = validate_planned_bounded_command(planned_command)
        auth = _json_safe(authorization, "authorization")
        if type(auth) is not dict or auth.get("confirmed_command_id") != command["query"]["command_id"]:
            raise ExecutionContractError("operator authorization does not match plan")
        mode = auth.get("mode")
        if mode not in {"QUALIFIED", "SUPERVISED_EXPERIMENTAL_TRIAL"}:
            raise ExecutionContractError("operator authorization mode is invalid")
        if auth.get("automatic_capability_qualification_changed") is not False:
            raise ExecutionContractError("operator authorization must not change capability qualification")
        if mode == "SUPERVISED_EXPERIMENTAL_TRIAL":
            _text(auth.get("experimental_reason"), "experimental_reason")
        elif auth.get("experimental_reason") is not None:
            raise ExecutionContractError("qualified authorization must not carry experimental_reason")
        url_root = _text(base_url, "base_url")
        if not math.isfinite(timeout_s) or timeout_s <= 0 or not math.isfinite(poll_interval_s) or poll_interval_s <= 0 or not math.isfinite(terminal_timeout_s) or terminal_timeout_s <= 0:
            raise ExecutionContractError("HTTP and polling timeouts must be positive finite values")
    except ExecutionContractError as error:
        return _record(result="BLOCKED_EXECUTION_CONTRACT", reason=str(error), planned_command=command, authorization=auth)

    try:
        preflight_code, preflight = request("GET", _bounded_url(url_root, "/status", {}), timeout_s)
    except ExecutionContractError as error:
        return _record(result="BLOCKED_PREFLIGHT", reason=str(error), planned_command=command, authorization=auth, network_performed=True)
    if preflight_code != 200:
        return _record(result="BLOCKED_PREFLIGHT", reason=f"STATUS_HTTP_{preflight_code}", planned_command=command, authorization=auth, preflight_status=preflight, network_performed=True)
    preflight_reason = _ready_status(preflight, command)
    if preflight_reason is not None:
        return _record(result="BLOCKED_PREFLIGHT", reason=preflight_reason, planned_command=command, authorization=auth, preflight_status=preflight, network_performed=True)

    try:
        response_code, response = request("POST", _bounded_url(url_root, command["endpoint"], command["query"]), timeout_s)
    except ExecutionContractError as error:
        return _record(result="COMMAND_TRANSPORT_UNKNOWN", reason=str(error), planned_command=command, authorization=auth, preflight_status=preflight, network_performed=True)
    if response_code != 202 or response.get("accepted") is not True or response.get("command_id") != command["query"]["command_id"]:
        return _record(result="COMMAND_NOT_ACCEPTED", reason=f"COMMAND_HTTP_{response_code}", planned_command=command, authorization=auth, preflight_status=preflight, request_response=response, network_performed=True)

    deadline = monotonic() + terminal_timeout_s
    terminal: dict[str, Any] | None = None
    while monotonic() <= deadline:
        sleep(poll_interval_s)
        try:
            status_code, status = request("GET", _bounded_url(url_root, "/status", {}), timeout_s)
        except ExecutionContractError as error:
            return _record(result="TERMINAL_STATUS_UNKNOWN", reason=str(error), planned_command=command, authorization=auth, preflight_status=preflight, request_response=response, network_performed=True, motor_command_performed=True)
        if status_code != 200:
            return _record(result="TERMINAL_STATUS_UNKNOWN", reason=f"STATUS_HTTP_{status_code}", planned_command=command, authorization=auth, preflight_status=preflight, request_response=response, terminal_status=status, network_performed=True, motor_command_performed=True)
        if status.get("boot_session_id") != command["query"]["boot_session_id"]:
            return _record(result="TERMINAL_STATUS_UNKNOWN", reason="BOOT_SESSION_CHANGED_DURING_COMMAND", planned_command=command, authorization=auth, preflight_status=preflight, request_response=response, terminal_status=status, network_performed=True, motor_command_performed=True)
        if status.get("last_command_id") == command["query"]["command_id"] and status.get("last_command_state") in TERMINAL_COMMAND_STATES:
            terminal = status
            break
    if terminal is None:
        return _record(result="TERMINAL_STATUS_TIMEOUT", reason="NO_RETRY_OR_CORRECTION_PERFORMED", planned_command=command, authorization=auth, preflight_status=preflight, request_response=response, network_performed=True, motor_command_performed=True)
    return _record(result="AWAIT_REOBSERVATION", reason=terminal.get("bounded_fault_reason"), planned_command=command, authorization=auth, preflight_status=preflight, request_response=response, terminal_status=terminal, network_performed=True, motor_command_performed=True)
