"""功能菜单：选择编号，等待本次任务完成，再选择下一项。"""
import math


def show_menu(points, tasks, modes, output=print):
    output('\nchairman_navigation 功能菜单')
    output('定点导航：')
    for number, point in points.items():
        if number not in tasks:
            output(f'  {number:>3}: {point[4]}（模式 {point[5]}：{modes[point[5]]["name"]}）')
    output('特殊功能：')
    for number, task in tasks.items():
        output(f'  {number:>3}: {task.get("description", task.get("name", number))}')
    output('输入功能编号；执行时 Ctrl+C 中止当前任务；输入 m 重看菜单；菜单中 q 或 Ctrl+C 退出。')


def menu_loop(points, tasks, modes, submit, stopped, active, read=input, output=print, progress=None):
    """同步等待本次请求完成，避免提前接收下一次选择。"""
    known_ids = set(points) | set(tasks)
    show_menu(points, tasks, modes, output)
    while not stopped.is_set():
        try:
            line = read('选择功能编号> ').strip()
            if line.lower() == 'q':
                return
            if line.lower() == 'm':
                show_menu(points, tasks, modes, output)
                continue
            if not line:
                continue
            target = int(line)
            if target not in known_ids:
                raise ValueError(f'没有功能 {target}，请按菜单选择')
            offset = (0.0, 0.0, 0.0)
            if tasks.get(target, {}).get('uses_offset', False):
                fields = read('输入偏置 x y z（单位 m，空格分隔）> ').split()
                if len(fields) != 3:
                    raise ValueError('偏置需要三个数，例如 0.2 0.0 0.0')
                offset = tuple(float(v) for v in fields)
                if not all(math.isfinite(v) for v in offset):
                    raise ValueError('偏置必须是有限数值')
            active.set()
            future = submit(target, offset)
            if progress is None:
                output(f'[任务已提交] 编号 {target}，等待执行结果；Ctrl+C 中止')
            while not future.done():
                if progress is not None:
                    progress.render()
                if stopped.wait(0.05):
                    return
            result = future.result()
            if progress is not None:
                progress.clear()
            if progress is None:
                output(f'{"完成" if result.success else "未完成"}：{result.message}')
            show_menu(points, tasks, modes, output)
        except EOFError:
            return
        except (ValueError, RuntimeError) as exc:
            output(str(exc))
        finally:
            if progress is not None:
                progress.clear()
            active.clear()


def main(args=None):
    from framework.core.runtime import run_runtime
    run_runtime(menu=menu_loop, args=args)


def server_main(args=None):
    """保留仅服务入口；与菜单共用 core 中的公共接口。"""
    from framework.core.runtime import run_runtime
    run_runtime(args=args)


if __name__ == '__main__':
    main()
