"""独立运行Main按真实分页节奏的通信测试，不运行其他套件。"""
from pathlib import Path


def main():
    """复用ARM编译执行工具，仅截取工具配置，跳过原套件的自动编译检查。"""
    path=Path(__file__).with_name('run_tests.py')
    source=path.read_text(encoding='utf-8')
    lines=[line for line in source.splitlines() if not line.startswith('subprocess.run([sys.executable,str(Path(__file__).with_name(')]
    prefix='\n'.join(lines).split("\nresult=run('supervisor'")[0]
    context={'__file__':str(path),'__name__':'__main__'}
    exec(compile(prefix,str(path),'exec'),context)
    result=context['run']('main-status-link',['tests/robot/main_status_link_test.cpp','RobotComponents/component/gimbal_chassis_comm.cpp'],
        ['tests/robot','RobotComponents/component','Chassis/inc'],['ROBOT_NORMAL_MAIN_ENABLED=1'])
    print('Main逐毫秒读取、3ms分页、安全状态心跳及双板状态机：',result,flush=True)
    if result:raise SystemExit(result)


if __name__=='__main__':main()
