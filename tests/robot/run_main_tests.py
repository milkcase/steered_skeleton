"""复用已有ARM运行器，只运行两板真实主程序参数回归。"""
from pathlib import Path
import ast,sys


def main():
    """提取公共编译器配置和每板源列表，不执行旧场景，只运行生产模式用例。"""
    path=Path(__file__).with_name('run_tests.py');source=path.read_text(encoding='utf-8')
    context={'__file__':str(path),'__name__':'__main__'}
    exec(compile(source.split("\nresult=run('supervisor'")[0],str(path),'exec'),context)
    loop=next(n for n in ast.parse(source).body if isinstance(n,ast.For) and isinstance(n.target,ast.Name) and n.target.id=='board')
    for board in ('Chassis','Gimbal'):
        context['board']=board
        for statement in loop.body:
            if isinstance(statement,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='result' for t in statement.targets):break
            exec(compile(ast.fix_missing_locations(ast.Module(body=[statement],type_ignores=[])),str(path),'exec'),context)
        if board=='Chassis':context['runtime_defines']+=['ROBOT_YAW_VELOCITY_FF=0.0123f']
        result=context['run'](board+'-main-runtime',context['runtime_sources'],context['runtime_includes'],
            context['runtime_defines']+['ROBOT_NORMAL_MAIN_ENABLED=1','TEST_PRODUCTION_MAIN'])
        print(board,'主程序真实参数测试：',result)
        if result:raise SystemExit(result)
        if board=='Chassis':
            result=context['run']('Chassis-main-peer-rejection',context['runtime_sources'],context['runtime_includes'],
                context['runtime_defines']+['ROBOT_NORMAL_MAIN_ENABLED=1','TEST_PRODUCTION_MAIN','TEST_MAIN_PEER_REJECTION'])
            print('对端拒收首因与CRC事件锁存、后续安全页不覆盖：',result)
            if result:raise SystemExit(result)
            result=context['run']('Chassis-main-axis-feedback-loss',context['runtime_sources'],context['runtime_includes'],
                context['runtime_defines']+['ROBOT_NORMAL_MAIN_ENABLED=1','TEST_PRODUCTION_MAIN','TEST_MAIN_AXIS_FEEDBACK_LOSS'])
            print('电机先禁用反馈：首因锁存、全组归零、右下保留：',result)
            if result:raise SystemExit(result)
            for name,extra in [('dual-main-runtime',[]),
                    ('dual-main-follow',['TEST_DUAL_FOLLOW','ROBOT_SPIN_TARGET_RPM=45.0f','ROBOT_SPIN_MAX_RPM=45.0f',
                        'ROBOT_SPIN_ACCEL_RPM_S=60.0f','ROBOT_SPIN_TRANSLATION_OFFSET_DEG=-8.0f','ROBOT_DRIVE_TARGET_SPEED_RPM=100.0f',
                        'ROBOT_YAW_AIM_RPM=30.0f','ROBOT_YAW_POSITION_SPEED_RPM=60.0f','ROBOT_YAW_TARGET_RATE_RPM=90.0f']),
                    ('dual-main-pitch-outside',['TEST_DUAL_PITCH_OUTSIDE','ROBOT_PITCH_POSITION_LIMIT_ENABLED=1']),
                    ('dual-main-pitch-free',['TEST_DUAL_PITCH_FREE','ROBOT_PITCH_POSITION_LIMIT_ENABLED=0'])]:
                result=context['run'](name,context['runtime_sources'],context['runtime_includes'],
                    context['runtime_defines']+['ROBOT_NORMAL_MAIN_ENABLED=1','TEST_DUAL_MAIN']+extra)
                print(name,'双板真实发送/独立时钟/软件角度开关：',result)
                if result:raise SystemExit(result)
        if board=='Gimbal':
            result=context['run'](board+'-main-old-parameters',context['runtime_sources'],context['runtime_includes'],
                context['runtime_defines']+['ROBOT_NORMAL_MAIN_ENABLED=1','TEST_PRODUCTION_MAIN',
                'ROBOT_PITCH_POSITION_KP=1.5f','ROBOT_PITCH_SPEED_KP=0.17f','ROBOT_DRIVE_TARGET_SPEED_RPM=90.0f',
                'ROBOT_PITCH_POSITION_LIMIT_ENABLED=1'])
            print('旧云台编译参数接收新底盘数值：',result)
            if result:raise SystemExit(result)
    result=context['run']('main-tuning-link',['tests/robot/main_tuning_test.cpp'],['RobotComponents/component','tests/robot'])
    print('整组参数提交/坏帧拒绝/ACK截止/只重启底盘：',result)
    if result:raise SystemExit(result)
    result=context['run']('pitch-feedback-link',['tests/robot/pitch_feedback_test.cpp'],['RobotComponents/component','tests/robot'])
    print('pitch完整闭环float快照/原子提交/CRC/重复帧/接收超时：',result)
    if result:raise SystemExit(result)
    result=context['run']('main-status-link',['tests/robot/main_status_link_test.cpp','RobotComponents/component/gimbal_chassis_comm.cpp'],
        ['tests/robot','RobotComponents/component','Chassis/inc'],['ROBOT_NORMAL_MAIN_ENABLED=1'])
    print('Main安全心跳连续、双板分页错相与恢复握手：',result)
    if result:raise SystemExit(result)


if __name__=='__main__':main()
