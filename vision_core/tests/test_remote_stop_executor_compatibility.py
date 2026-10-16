"""No network: asynchronous stop remains nonterminal until matching command completes."""
from sie_core.supervised_bounded_executor import execute_one_supervised_command


def test_bounded_executor_waits_through_remote_braking():
    plan = {'result':'PLANNED_BOUNDED_COMMAND','network_performed':False,
            'method':'POST','endpoint':'/move-forward',
            'query':{'boot_session_id':'0123456789ABCDEF','command_id':'stop-compat-1','distance_m':'0.1'}}
    auth = {'mode':'SUPERVISED_EXPERIMENTAL_TRIAL','confirmed_command_id':'stop-compat-1',
            'experimental_reason':'offline compatibility test','automatic_capability_qualification_changed':False}
    base = {'motion_api_version':'v2_bounded_motion_api','boot_session_id':'0123456789ABCDEF',
            'bounded_fault_latched':False,'state':'READY','active_command_id':None}
    replies = [(200,base), (202,{'accepted':True,'command_id':'stop-compat-1'}),
               (200,{**base,'state':'BRAKING','active_command_id':'stop-compat-1',
                     'remote_stop_pending':True,'last_command_id':'previous', 'last_command_state':'STOPPED'}),
               (200,{**base,'remote_stop_pending':False,'last_command_id':'stop-compat-1',
                     'last_command_state':'STOPPED','reobserve_required':True})]
    calls=[]
    clock=[0.0]
    def request(method,url,timeout):
        calls.append((method,url))
        return replies.pop(0)
    def sleep(seconds): clock[0]+=seconds
    result=execute_one_supervised_command(planned_command=plan,authorization=auth,
            base_url='http://127.0.0.1',request=request,monotonic=lambda:clock[0],sleep=sleep)
    assert result['result']=='AWAIT_REOBSERVATION'
    assert result['terminal_status']['last_command_state']=='STOPPED'
    assert result['reobserve_required'] is True
    assert [m for m,u in calls]==['GET','POST','GET','GET']
    assert all('/stop' not in u for m,u in calls)
    assert not replies
