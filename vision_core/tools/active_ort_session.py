"""Resolve and configure the actually executed ONNX session, never outer shadows."""
import hashlib
from pathlib import Path

def active_session_owner(backend):
    """Follow the actual AutoBackend.forward delegation, not shadow attributes."""
    seen=set();owner=backend
    while getattr(owner,'backend',None) is not None:
        if id(owner) in seen:raise ValueError('backend delegation cycle')
        seen.add(id(owner));owner=owner.backend
    session=getattr(owner,'session',None)
    if session is None or not callable(getattr(session,'run',None)):
        raise ValueError('delegated backend has no supported ORT session.run')
    return owner


def active_session_metadata(adapter):
    outer=adapter.model.predictor.model;owner=active_session_owner(outer);session=owner.session
    opts=session.get_session_options()
    return {'owner':type(owner).__module__+'.'+type(owner).__qualname__,
        'delegated':owner is not outer,
        'outer_session_is_active':getattr(outer,'session',None) is session,
        'providers':session.get_providers(),'input_shape':session.get_inputs()[0].shape,
        'intra_op_num_threads':opts.intra_op_num_threads,'inter_op_num_threads':opts.inter_op_num_threads,
        'execution_mode':str(opts.execution_mode),
        'meaning':'Actual session options; zero threads means ORT default, not zero execution threads.'}



def configure_active_ort(adapter,model_path,threads):
    if threads not in (1,2):raise ValueError('unsupported active ORT thread count')
    import onnxruntime as ort
    owner=active_session_owner(adapter.model.predictor.model)
    previous=owner.session
    shape=previous.get_inputs()[0].shape
    if previous.get_providers()!=['CPUExecutionProvider']:
        raise ValueError('active ORT provider is not CPU-only')
    # Adapter has already verified reviewed export SHA and package versions.
    if hashlib.sha256(Path(model_path).read_bytes()).hexdigest()!=adapter.metadata['model_sha256']:
        raise ValueError('active export SHA mismatch')
    options=ort.SessionOptions();options.intra_op_num_threads=threads;options.inter_op_num_threads=1
    options.execution_mode=ort.ExecutionMode.ORT_SEQUENTIAL
    options.add_session_config_entry('session.intra_op.allow_spinning','0')
    candidate=ort.InferenceSession(str(model_path),sess_options=options,providers=['CPUExecutionProvider'])
    if candidate.get_inputs()[0].shape!=shape or candidate.get_providers()!=['CPUExecutionProvider']:
        raise ValueError('active ORT shape/provider changed')
    owner.session=candidate
    verify_active_ort(adapter,threads)
    return active_session_metadata(adapter)


def verify_active_ort(adapter,threads):
    session=active_session_owner(adapter.model.predictor.model).session
    options=session.get_session_options()
    if (options.intra_op_num_threads!=threads or options.inter_op_num_threads!=1
            or options.get_session_config_entry('session.intra_op.allow_spinning')!='0'
            or session.get_providers()!=['CPUExecutionProvider']):
        raise ValueError('requested active ORT settings were not applied')
