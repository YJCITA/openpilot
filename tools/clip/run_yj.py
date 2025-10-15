#!/usr/bin/env python3

import atexit
import logging
import os
import platform
import re
import select
import shutil
import sys
import threading
import time
from argparse import ArgumentParser, ArgumentTypeError
from collections.abc import Sequence
from pathlib import Path
from random import randint
from subprocess import Popen, PIPE
from typing import Literal

from cereal.messaging import SubMaster
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.tools.lib.route import Route
from openpilot.tools.lib.logreader import LogReader

DEFAULT_OUTPUT = 'output.mp4'
DEMO_START = 90
DEMO_END = 105
DEMO_ROUTE = 'a2a0ccea32023010/2023-07-27--13-01-19'
FRAMERATE = 20
PIXEL_DEPTH = '24'
RESOLUTION = '2160x1080'
SECONDS_TO_WARM = 2
PROC_WAIT_SECONDS = 30*10

OPENPILOT_FONT = str(Path(BASEDIR, 'selfdrive/assets/fonts/Inter-Regular.ttf').resolve())
REPLAY = str(Path(BASEDIR, 'tools/replay/replay').resolve())
UI = str(Path(BASEDIR, 'selfdrive/ui/ui').resolve())

logger = logging.getLogger('clip.py')


def check_for_failure(proc: Popen):
  exit_code = proc.poll()
  if exit_code is not None and exit_code != 0:
    cmd = str(proc.args)
    if isinstance(proc.args, str):
      cmd = proc.args
    elif isinstance(proc.args, Sequence):
      cmd = str(proc.args[0])
    msg = f'{cmd} failed, exit code {exit_code}'
    logger.error(msg)

    # 捕获输出
    try:
      stdout, stderr = proc.communicate(timeout=2)
      if stdout:
        logger.error(f"stdout: {stdout.decode('utf-8', errors='ignore')}")
      if stderr:
        logger.error(f"stderr: {stderr.decode('utf-8', errors='ignore')}")
    except:
      logger.error("无法读取进程输出")

    raise ChildProcessError(msg)


def escape_ffmpeg_text(value: str):
  special_chars = {',': '\\,', ':': '\\:', '=': '\\=', '[': '\\[', ']': '\\]'}
  value = value.replace('\\', '\\\\\\\\\\\\\\\\')
  for char, escaped in special_chars.items():
    value = value.replace(char, escaped)
  return value


def get_logreader(route: Route):
  if len(route.qlog_paths()):
    segment_name_list = route.qlog_paths()
    for item in segment_name_list:
      if item is not None:
        segment_name = item
  else:
     segment_name = route.name.canonical_name
  return LogReader(segment_name)
  # return LogReader(route.qlog_paths()[0] if len(route.qlog_paths()) else route.name.canonical_name)


def get_meta_text(lr: LogReader, route: Route):
  init_data = lr.first('initData')
  car_params = lr.first('carParams')
  origin_parts = init_data.gitRemote.split('|')
  origin = origin_parts[3] if len(origin_parts) > 3 else 'unknown'
  return ', '.join([
    f"openpilot v{init_data.version}",
    f"route: {route.name.canonical_name}",
    f"car: {None if car_params is None else car_params.carFingerprint}",
    f"origin: {origin}",
    f"branch: {init_data.gitBranch}",
    f"commit: {init_data.gitCommit[:7]}",
    f"modified: {str(init_data.dirty).lower()}",
  ])


def parse_args(parser: ArgumentParser):
  args = parser.parse_args()
  if args.demo:
    args.route = DEMO_ROUTE
    if args.start is None or args.end is None:
      args.start = DEMO_START
      args.end = DEMO_END
  elif args.route.count('/') == 1:
    # -YJ- 其中/换为|
    args.route = args.route.replace('/', '|')
    # 如果不传 start 和 end，先创建 Route 对象获取长度，然后自动设置
    if args.start is None or args.end is None:
      # 临时创建 Route 对象来获取数据长度
      temp_route = Route(args.route, data_dir=args.data_dir)
      route_length = round(temp_route.max_seg_number * 60)

      if args.start is None:
        args.start = SECONDS_TO_WARM  # 从预热时间开始
      if args.end is None:
        args.end = route_length       # 到数据结束

  elif args.route.count('/') == 3:
    if args.start is not None or args.end is not None:
      parser.error('don\'t provide timing when including it in the route ID')
    parts = args.route.split('/')
    args.route = '/'.join(parts[:2])
    args.start = int(parts[2])
    args.end = int(parts[3])
  if args.end <= args.start:
    parser.error(f'end ({args.end}) must be greater than start ({args.start})')
  if args.start < SECONDS_TO_WARM:
    parser.error(f'start must be greater than {SECONDS_TO_WARM}s to allow the UI time to warm up')

  # 创建最终的 Route 对象
  args.route = Route(args.route, data_dir=args.data_dir)

  # 验证时间范围
  length = round(args.route.max_seg_number * 60)
  if args.start >= length:
    parser.error(f'start ({args.start}s) cannot be after end of route ({length}s)')
  if args.end > length:
    parser.error(f'end ({args.end}s) cannot be after end of route ({length}s)')

  return args


def populate_car_params(lr: LogReader):
  init_data = lr.first('initData')
  assert init_data is not None

  params = Params()
  entries = init_data.params.entries
  for cp in entries:
    key, value = cp.key, cp.value
    try:
      converted_value = params.cpp2python(key, value)
      # Skip None values for JSON type parameters to avoid type mismatch errors
      if converted_value is not None:
        params.put(key, converted_value)
      else:
        logger.debug(f"skipping None value for param '{key}'")
    except UnknownKeyName:
      # forks of openpilot may have other Params keys configured. ignore these
      logger.warning(f"unknown Params key '{key}', skipping")
  logger.debug('persisted CarParams')


def start_proc(args: list[str], env: dict[str, str], capture_output=True):
  if capture_output:
    # 打开 stdin 以便优雅退出时写入 'q' 给 ffmpeg
    return Popen(args, env=env, stdin=PIPE, stdout=PIPE, stderr=PIPE)
  else:
    return Popen(args, env=env, stdin=PIPE, stdout=PIPE, stderr=PIPE, universal_newlines=True, bufsize=1)


def debug_ffmpeg_error(proc: Popen):
  """调试 ffmpeg 错误输出"""
  if proc.stderr:
    stderr_output = proc.stderr.read().decode('utf-8', errors='ignore')
    if stderr_output:
      logger.error(f"ffmpeg stderr: {stderr_output}")
  if proc.stdout:
    stdout_output = proc.stdout.read().decode('utf-8', errors='ignore')
    if stdout_output:
      logger.debug(f"ffmpeg stdout: {stdout_output}")


def validate_env(parser: ArgumentParser):
  if platform.system() not in ['Linux']:
    parser.exit(1, f'clip.py: error: {platform.system()} is not a supported operating system\n')
  for proc in ['Xvfb', 'ffmpeg']:
    if shutil.which(proc) is None:
      parser.exit(1, f'clip.py: error: missing {proc} command, is it installed?\n')
  for proc in [REPLAY, UI]:
    if shutil.which(proc) is None:
      parser.exit(1, f'clip.py: error: missing {proc} command, did you build openpilot yet?\n')


def validate_output_file(output_file: str):
  if not output_file.endswith('.mp4'):
    raise ArgumentTypeError('output must be an mp4')
  # Expand ~ to user home directory
  expanded_path = os.path.expanduser(output_file)
  # Create output directory if it doesn't exist
  output_dir = os.path.dirname(expanded_path)
  if output_dir and not os.path.exists(output_dir):
    os.makedirs(output_dir, exist_ok=True)
  return expanded_path


def validate_route(route: str):
  if route.count('/') not in (1, 3):
    raise ArgumentTypeError(f'route must include or exclude timing, example: {DEMO_ROUTE}')
  return route


def validate_title(title: str):
  if len(title) > 80:
    raise ArgumentTypeError('title must be no longer than 80 chars')
  return title


def wait_for_frames(procs: list[Popen]):
  sm = SubMaster(['uiDebug'])
  no_frames_drawn = True
  while no_frames_drawn:
    sm.update()
    no_frames_drawn = sm['uiDebug'].drawTimeMillis == 0.
    for proc in procs:
      check_for_failure(proc)


def monitor_ffmpeg_progress(proc: Popen, duration: int):
  """监控 ffmpeg 进度并实时显示"""
  last_progress = {'time': 0, 'frame': 0, 'fps': 0, 'speed': 0, 'size': 0}
  last_report_time = 0
  report_interval = 1.0  # 每1秒报告一次进度

  try:
    while proc.poll() is None:
      # 使用 select 来非阻塞读取 stderr，减少超时时间
      if proc.stderr and select.select([proc.stderr], [], [], 0.05)[0]:
        line_bytes = proc.stderr.readline()
        if not line_bytes:
          break

        # 解码为字符串
        try:
          line = line_bytes.decode('utf-8', errors='ignore')
        except:
          continue

        # 解析 ffmpeg 输出: frame= 1234 fps=60 ... time=00:01:23.45 ... speed=1.2x
        if 'frame=' in line:
          # 提取帧数
          frame_match = re.search(r'frame=\s*(\d+)', line)
          if frame_match:
            last_progress['frame'] = int(frame_match.group(1))

          # 提取 fps
          fps_match = re.search(r'fps=\s*([\d.]+)', line)
          if fps_match:
            last_progress['fps'] = float(fps_match.group(1))

          # 提取时间 (格式: 00:01:23.45)
          time_match = re.search(r'time=(\d+):(\d+):([\d.]+)', line)
          if time_match:
            hours = int(time_match.group(1))
            minutes = int(time_match.group(2))
            seconds = float(time_match.group(3))
            current_time = hours * 3600 + minutes * 60 + seconds
            last_progress['time'] = current_time

          # 提取速度
          speed_match = re.search(r'speed=\s*([\d.]+)x', line)
          if speed_match:
            last_progress['speed'] = float(speed_match.group(1))

          # 提取文件大小
          size_match = re.search(r'size=\s*(\d+)kB', line)
          if size_match:
            last_progress['size'] = int(size_match.group(1))

          # 实时显示进度（每1秒或进度变化明显时）
          current_time = time.time()
          if last_progress['time'] > 0 and duration > 0:
            progress_pct = min(100, (last_progress['time'] / duration) * 100)

            # 每1秒或进度变化超过5%时显示
            if (current_time - last_report_time >= report_interval or
                abs(progress_pct - last_report_time) >= 5):

              elapsed_str = f"{int(last_progress['time']//60):02d}:{int(last_progress['time']%60):02d}"
              duration_str = f"{int(duration//60):02d}:{int(duration%60):02d}"

              # 使用 print 在同一行显示进度（\r 覆盖当前行）
              progress_line = (
                f"渲染进度: {progress_pct:5.1f}% | "
                f"时间: {elapsed_str}/{duration_str} | "
                f"帧: {last_progress['frame']:5d} | "
                f"FPS: {last_progress['fps']:4.1f} | "
                f"速度: {last_progress['speed']:.2f}x | "
                f"大小: {last_progress['size']/1024:.1f}MB"
              )
              print(f"\r{progress_line}", end="", flush=True)
              last_report_time = current_time

      else:
        # 如果没有新数据，短暂休眠
        time.sleep(0.05)

  except Exception as e:
    logger.debug(f"监控 ffmpeg 进度时出错: {e}")

  return last_progress


def clip(
  data_dir: str | None,
  quality: Literal['low', 'balanced', 'high'],
  prefix: str,
  route: Route,
  out: str,
  start: int,
  end: int,
  speed: int,
  target_mb: int,
  title: str | None,
):
  logger.info(f'clipping route {route.name.canonical_name}, start={start} end={end} quality={quality}')
  lr = get_logreader(route)

  begin_at = max(start - SECONDS_TO_WARM, 0)
  duration = end - start
  # 移除文件大小限制，使用固定质量控制
  # bit_rate_kbps = int(round(target_mb * 8 * 1024 * 1024 / duration / 1000 * 1.2))

  # TODO: evaluate creating fn that inspects /tmp/.X11-unix and creates unused display to avoid possibility of collision
  display = f':{randint(99, 999)}'

  box_style = 'box=1:boxcolor=black@0.33:boxborderw=7'
  meta_text = get_meta_text(lr, route)
  print(f"quality: {quality}")
  # 统一质量配置，同时控制数据源和编码质量
  if quality == 'low':
    crf_value = '23'
    preset_value = 'fast'        # 稍微慢一点，质量更好
    tune_value = 'film'
    max_bitrate = '1000k'        # 固定最大比特率
    base_filters = []
  elif quality == 'high':
    crf_value = '18'             # 更高质量
    preset_value = 'medium'      # 平衡质量和速度
    tune_value = 'film'
    max_bitrate = '50000k'       # 更高比特率上限
    base_filters = []
  else:  # balanced
    crf_value = '20'             # 平衡质量
    preset_value = 'fast'        # 较快编码
    tune_value = 'film'
    max_bitrate = '2000k'       # 中等比特率上限
    base_filters = []

  overlays = [
    # metadata overlay
    f"drawtext=text='{escape_ffmpeg_text(meta_text)}':fontfile={OPENPILOT_FONT}:fontcolor=white:fontsize=15:{box_style}:x=(w-text_w)/2:y=5.5:enable='between(t,1,5)'",
    # route time overlay
    f"drawtext=text='%{{eif\\:floor(({start}+t)/60)\\:d\\:2}}\\:%{{eif\\:mod({start}+t\\,60)\\:d\\:2}}':fontfile={OPENPILOT_FONT}:fontcolor=white:fontsize=24:{box_style}:x=w-text_w-38:y=38"
  ]

  # 合并所有滤镜
  all_filters = base_filters + overlays
  if title:
    overlays.append(f"drawtext=text='{escape_ffmpeg_text(title)}':fontfile={OPENPILOT_FONT}:fontcolor=white:fontsize=32:{box_style}:x=(w-text_w)/2:y=53")

  if speed > 1:
    all_filters += [
      f"setpts=PTS/{speed}",
      "fps=60",
    ]

  # 固定质量配置，不依赖文件大小限制
  # 添加 -progress 参数确保实时输出进度信息
  ffmpeg_cmd = [
    'ffmpeg', '-y',
    '-video_size', RESOLUTION,
    '-framerate', str(FRAMERATE),
    '-f', 'x11grab',
    '-rtbufsize', '100M',
    '-draw_mouse', '0',
    '-i', display,
    '-c:v', 'libx264',
    '-crf', crf_value,              # 使用CRF恒定质量
    '-maxrate', max_bitrate,        # 固定最大比特率
    '-bufsize', f'{int(max_bitrate[:-1])*2}k',  # 缓冲区大小 (2倍最大比特率)
    '-filter:v', ','.join(overlays),
    '-preset', preset_value,
    '-tune', tune_value,
    '-pix_fmt', 'yuv420p',
    '-movflags', '+faststart',
    '-progress', 'pipe:2',  # 强制实时输出进度到stderr
    '-f', 'mp4',
    '-t', str(duration),
    out,
  ]

  replay_cmd = [REPLAY, '--ecam', '-c', '1', '-s', str(begin_at), '--prefix', prefix]
  if data_dir:
    replay_cmd.extend(['--data_dir', data_dir])
  # 统一质量控制：low 使用 qcam，balanced/high 使用 hevc
  if quality == 'low':
    replay_cmd.append('--qcam')
  replay_cmd.append(route.name.canonical_name)

  ui_cmd = [UI, '-platform', 'xcb']
  xvfb_cmd = ['Xvfb', display, '-terminate', '-screen', '0', f'{RESOLUTION}x{PIXEL_DEPTH}']

  with OpenpilotPrefix(prefix, shared_download_cache=True):
    populate_car_params(lr)

    env = os.environ.copy()
    env['DISPLAY'] = display

    xvfb_proc = start_proc(xvfb_cmd, env)
    atexit.register(lambda: xvfb_proc.terminate())
    ui_proc = start_proc(ui_cmd, env)
    atexit.register(lambda: ui_proc.terminate())
    replay_proc = start_proc(replay_cmd, env)
    atexit.register(lambda: replay_proc.terminate())
    procs = [replay_proc, ui_proc, xvfb_proc]

    logger.info('waiting for replay to begin (loading segments, may take a while)...')
    wait_for_frames(procs)

    logger.debug(f'letting UI warm up ({SECONDS_TO_WARM}s)...')
    time.sleep(SECONDS_TO_WARM)
    for proc in procs:
      check_for_failure(proc)

    ffmpeg_proc = start_proc(ffmpeg_cmd, env, capture_output=True)
    procs.append(ffmpeg_proc)

    # 定义清理函数，确保异常时也能正确关闭ffmpeg
    def cleanup_ffmpeg():
      if ffmpeg_proc.poll() is None:
        logger.info('正在优雅地关闭 ffmpeg...')
        try:
          # 方案1：向 ffmpeg 发送 'q'（优雅退出，保证 mp4 可播放）
          if ffmpeg_proc.stdin:
            ffmpeg_proc.stdin.write(b'q')
            ffmpeg_proc.stdin.flush()
          ffmpeg_proc.wait(timeout=5)
          logger.info('ffmpeg 已正常关闭')
          return
        except Exception:
          logger.debug('写入 q 失败，尝试 terminate')
        ffmpeg_proc.terminate()
        try:
          ffmpeg_proc.wait(timeout=5)
          logger.info('ffmpeg 已正常关闭')
        except:
          logger.warning('ffmpeg 未响应，强制关闭')
          ffmpeg_proc.kill()

    atexit.register(cleanup_ffmpeg)

    logger.info(f'开始录制 ({duration}s)...')

    # 在单独的线程中监控 ffmpeg 进度
    progress_thread = threading.Thread(
      target=monitor_ffmpeg_progress,
      args=(ffmpeg_proc, duration),
      daemon=True
    )
    progress_thread.start()

    try:
      # 等待录制完成
      start_time = time.time()
      timeout = duration + PROC_WAIT_SECONDS

      while time.time() - start_time < timeout:
        # 检查 ffmpeg 是否已结束
        if ffmpeg_proc.poll() is not None:
          break

        # 检查其他进程是否失败
        for proc in procs[:-1]:  # 不包括 ffmpeg_proc
          check_for_failure(proc)

        time.sleep(0.5)

      # 等待 ffmpeg 完全结束
      if ffmpeg_proc.poll() is None:
        logger.warning(f'超时等待 ffmpeg 结束，正在终止...')
        cleanup_ffmpeg()

      # 等待进度监控线程结束
      progress_thread.join(timeout=2)

      # 最终检查所有进程
      for proc in procs:
        check_for_failure(proc)

      # 录制完成，换行并显示最终结果
      print()  # 换行，结束进度显示行
      logger.info(f'✅ 录制完成: {Path(out).resolve()}')

    except KeyboardInterrupt:
      logger.info('⚠️  检测到用户中断，正在保存已录制的视频...')
      cleanup_ffmpeg()
      progress_thread.join(timeout=2)
      logger.info(f'✅ 已保存部分视频: {Path(out).resolve()}')
      raise
    except Exception as e:
      logger.error(f'❌ 录制过程中出错: {e}')
      cleanup_ffmpeg()
      progress_thread.join(timeout=2)
      raise


def main():
  p = ArgumentParser(prog='clip.py', description='clip your openpilot route.', epilog='comma.ai')
  validate_env(p)
  route_group = p.add_mutually_exclusive_group(required=True)
  route_group.add_argument('route', nargs='?', type=validate_route, help=f'The route (e.g. {DEMO_ROUTE} or {DEMO_ROUTE}/{DEMO_START}/{DEMO_END})')
  route_group.add_argument('--demo', help='use the demo route', action='store_true')
  p.add_argument('-d', '--data-dir', help='local directory where route data is stored')
  p.add_argument('-e', '--end', help='stop clipping at <end> seconds (default: render to end)', type=int)
  p.add_argument('-f', '--file-size', help='target file size (Discord/GitHub support max 10MB, default is 9MB)', type=float, default=9.)
  # p.add_argument('-o', '--output', help='output clip to (.mp4)', type=validate_output_file, default=DEFAULT_OUTPUT)
  p.add_argument('-o', '--output', help='output clip to (.mp4)', type=str, default=None)
  p.add_argument('-p', '--prefix', help='openpilot prefix', default=f'clip_{randint(100, 99999)}')
  p.add_argument('-q', '--quality', help='overall quality (low/balanced/high)', choices=['low', 'balanced', 'high'], default='balanced')
  p.add_argument('-x', '--speed', help='record the clip at this speed multiple', type=int, default=1)
  p.add_argument('-s', '--start', help='start clipping at <start> seconds (default: render from beginning)', type=int)
  p.add_argument('-t', '--title', help='overlay this title on the video (e.g. "Chill driving across the Golden Gate Bridge")', type=validate_title)
  args = parse_args(p)

  #
  if args.output is None:
    # 取本地时间命名
    time_str = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    # name = args.route.split("/")[-1]
    # args.output = f'{args.output}_{name}_{args.start}_{args.end}.mp4'
    args.output = os.path.join(args.data_dir, f'{time_str}.mp4')
  exit_code = 1
  try:
    clip(
      data_dir=args.data_dir,
      quality=args.quality,
      prefix=args.prefix,
      route=args.route,
      out=args.output,
      start=args.start,
      end=args.end,
      speed=args.speed,
      target_mb=args.file_size,
      title=args.title,
    )
    exit_code = 0
  except KeyboardInterrupt as e:
    logger.exception('interrupted by user', exc_info=e)
  except Exception as e:
    logger.exception('encountered error', exc_info=e)
  finally:
    atexit._run_exitfuncs()
    sys.exit(exit_code)


if __name__ == '__main__':
  logging.basicConfig(level=logging.INFO, format='%(asctime)s %(name)s %(levelname)s\t%(message)s')
  main()
