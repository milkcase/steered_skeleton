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
        result=context['run'](board+'-main-runtime',context['runtime_sources'],context['runtime_includes'],
            context['runtime_defines']+['ROBOT_NORMAL_MAIN_ENABLED=1','TEST_PRODUCTION_MAIN'])
        print(board,'主程序真实参数测试：',result)
        if result:raise SystemExit(result)
        if board=='Gimbal':
            result=context['run'](board+'-main-old-parameters',context['runtime_sources'],context['runtime_includes'],
                context['runtime_defines']+['ROBOT_NORMAL_MAIN_ENABLED=1','TEST_PRODUCTION_MAIN',
                'ROBOT_PITCH_POSITION_KP=1.5f','ROBOT_PITCH_SPEED_KP=0.17f','ROBOT_DRIVE_TARGET_SPEED_RPM=90.0f'])
            print('旧云台编译参数接收新底盘数值：',result)
            if result:raise SystemExit(result)
    result=context['run']('main-tuning-link',['tests/robot/main_tuning_test.cpp'],['RobotComponents/component','tests/robot'])
    print('整组参数提交/坏帧拒绝/ACK截止/只重启底盘：',result)
    if result:raise SystemExit(result)
    result=context['run']('main-status-link',['tests/robot/main_status_link_test.cpp','RobotComponents/component/gimbal_chassis_comm.cpp'],
        ['tests/robot','RobotComponents/component','Chassis/inc'],['ROBOT_NORMAL_MAIN_ENABLED=1'])
    print('Main安全心跳连续、双板分页错相与恢复握手：',result)
    if result:raise SystemExit(result)


if __name__=='__main__':main()
