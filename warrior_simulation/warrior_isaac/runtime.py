#!/usr/bin/env python3
"""Bootstrap the project's isolated Isaac runtime without loading shared installs."""

import importlib
import importlib.util
import os
from pathlib import Path
import runpy
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
PREFIX = ROOT / '.pixi/envs/isaac'
RUNTIME_ROOT = ROOT / '.pixi/isaac-runtime'
SCRIPTS = {'robot': 'run_robot.py', 'course': 'run_course.py', 'validate': 'validate_stage.py'}


def application_arguments():
    """Keep Kit configuration, logs and caches inside the project, leaving HOME intact."""
    RUNTIME_ROOT.mkdir(parents=True, exist_ok=True)
    # Warp initializes while SimulationApp loads its extensions and supports
    # this cache override independently of Kit's portable paths.
    os.environ['WARP_CACHE_PATH'] = str(RUNTIME_ROOT / 'cache/warp')
    arguments = ['--portable', '--portable-root', str(RUNTIME_ROOT)]
    # Portable mode leaves Omniverse's global paths and document configs shared.
    # These tokens redirect runtime storage and document/extension paths. Kit
    # may still inspect host system configs before applying token overrides.
    for token, folder in {
        'omni_global_cache': 'cache', 'omni_global_data': 'data',
        'omni_global_logs': 'logs', 'omni_global_config': 'omni_configs',
        'omni_documents': 'documents', 'shared_documents': 'documents/shared',
        'app_documents': 'documents/app', 'documents': 'documents/app',
    }.items():
        arguments.append(f'--/app/tokens/{token}={RUNTIME_ROOT / folder}')
    arguments.extend([
        # Installed, pinned extensions do not need hot reload. Watching their
        # entire cache can exhaust the host's inotify quota during startup.
        '--/app/extensions/fsWatcherEnabled=false',
        # This renderer setting otherwise uses the global Omniverse cache.
        f'--/rtx-transient/resourcemanager/localTextureCachePath={RUNTIME_ROOT / "cache/texturecache"}',
    ])
    return arguments


def isolated_isaac_root():
    """Locate the wheel without importing it or implicitly accepting NVIDIA's EULA."""
    if Path(sys.prefix).resolve() != PREFIX.resolve() or sys.version_info[:2] != (3, 11):
        raise RuntimeError('Use launch_robot.sh, launch_course.sh or validate_stage.sh; '
                           'Isaac requires this project\'s .pixi/envs/isaac Python 3.11 environment.')
    spec = importlib.util.find_spec('isaacsim')
    if spec is None or not spec.origin:
        raise RuntimeError('Isaac Sim is not installed. Run "pixi run install-isaac" in this project.')
    package = Path(spec.origin).resolve().parent
    if not package.is_relative_to(PREFIX.resolve()):
        raise RuntimeError(f'Refusing Isaac Sim outside the project environment: {package}')
    return package


def usd_environment(package, prefix, inherited):
    """Expose the pinned wheel's OpenUSD libraries without starting Kit or a GPU."""
    candidates = sorted(path for path in (package / 'extscache').glob('omni.usd.libs-*')
                        if (path / 'pxr').is_dir() and (path / 'bin').is_dir())
    if len(candidates) != 1:
        raise RuntimeError('Expected one bundled OpenUSD extension in the project Isaac environment; '
                           'run "pixi run install-isaac" to install the locked extension cache.')
    usd = candidates[0]
    environment = inherited.copy()
    environment['PYTHONPATH'] = str(usd)
    # The conda interpreter supplies libpython; the extension supplies USD,
    # Boost and TBB. Never append inherited ROS or another Isaac installation.
    environment['LD_LIBRARY_PATH'] = os.pathsep.join(str(path) for path in
        (usd / 'bin', usd / 'bin/deps', prefix / 'lib') if path.is_dir())
    environment['PYTHONNOUSERSITE'] = '1'
    return environment


def eula_accepted(package, environment):
    """Read NVIDIA's acceptance state; never create or modify its marker."""
    accepted = ('y', 'yes', '1')
    if environment.get('OMNI_KIT_ACCEPT_EULA', '').lower() in accepted:
        return True
    marker = package / 'kit/EULA_ACCEPTED'
    if marker.is_file():
        with marker.open(encoding='utf-8') as stream:
            return stream.readline().strip().lower() in accepted
    return False


def require_eula(package, interactive):
    if not interactive and not eula_accepted(package, os.environ):
        raise RuntimeError('NVIDIA\'s EULA has not been accepted for this project runtime. '
                           'Run "pixi run configure-isaac" in an interactive terminal, '
                           'review NVIDIA\'s agreement and answer its prompt before launching through ROS.')


def configure(package):
    """Let NVIDIA's own interactive prompt record acceptance in this environment."""
    require_eula(package, sys.stdin.isatty())
    try:
        importlib.import_module('isaacsim')
    except SystemExit:
        if not eula_accepted(package, os.environ):
            raise SystemExit('NVIDIA EULA was not accepted; Isaac runtime is not configured.')
        raise
    print(f'Isaac runtime configured: {PREFIX}')


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in (*SCRIPTS, 'configure'):
        raise SystemExit('Usage: run_isaac.sh {robot|course|validate|configure} [script arguments]')
    mode = sys.argv[1]
    try:
        package = isolated_isaac_root()
        if mode == 'configure':
            configure(package)
            return
        arguments = [str(HERE / SCRIPTS[mode]), *sys.argv[2:]]
        if mode == 'validate':
            # LD_LIBRARY_PATH must be established before Python loads .so files.
            os.execve(sys.executable, [sys.executable, *arguments],
                      usd_environment(package, PREFIX, os.environ))
        if not any(argument in ('-h', '--help') for argument in sys.argv[2:]):
            require_eula(package, sys.stdin.isatty())
        sys.argv = arguments
        runpy.run_path(arguments[0], run_name='__main__')
    except RuntimeError as error:
        raise SystemExit(str(error)) from error


if __name__ == '__main__':
    main()
