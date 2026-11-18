"""只运行两板真实Runtime/发送层和电机使能反馈联动的ARM测试。"""
from pathlib import Path
import ast


def main():
    """复用底盘编译清单，同时构造云台Runtime；测试不连接真实电机。"""
    path=Path(__file__).with_name('run_tests.py');source=path.read_text(encoding='utf-8')
    lines=[line for line in source.splitlines() if not line.startswith('subprocess.run([sys.executable,str(Path(__file__).with_name(')]
    context={'__file__':str(path),'__name__':'__main__'}
    exec(compile('\n'.join(lines).split("\nresult=run('supervisor'")[0],str(path),'exec'),context)
    loop=next(n for n in ast.parse(source).body if isinstance(n,ast.For) and isinstance(n.target,ast.Name) and n.target.id=='board')
    context['board']='Chassis'
    for statement in loop.body:
        if isinstance(statement,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='result' for t in statement.targets):break
        exec(compile(ast.fix_missing_locations(ast.Module(body=[statement],type_ignores=[])),str(path),'exec'),context)
    for name,extra in [('dual-main-runtime',[]),
            ('dual-main-pitch-outside',['TEST_DUAL_PITCH_OUTSIDE','ROBOT_PITCH_POSITION_LIMIT_ENABLED=1']),
            ('dual-main-pitch-free',['TEST_DUAL_PITCH_FREE','ROBOT_PITCH_POSITION_LIMIT_ENABLED=0'])]:
        result=context['run'](name,context['runtime_sources'],context['runtime_includes'],
            context['runtime_defines']+['ROBOT_NORMAL_MAIN_ENABLED=1','TEST_DUAL_MAIN']+extra)
        print(name,'双板真实Runtime/实际使能反馈/独立时钟/安全区：',result,flush=True)
        if result:raise SystemExit(result)


if __name__=='__main__':main()
