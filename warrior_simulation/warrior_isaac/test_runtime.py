"""Launcher isolation regressions; no Isaac installation, GPU or ROS is required."""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import tomllib
import types
import unittest
from unittest.mock import patch

import runtime

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


class LauncherTest(unittest.TestCase):
    def test_every_launcher_selects_project_environment_and_scrubs_inherited_paths(self):
        with tempfile.TemporaryDirectory(prefix='isaac launcher ') as directory:
            temporary = Path(directory)
            pixi = temporary / 'pixi'
            pixi.write_text(f'#!{sys.executable}\nimport json, os, sys\n'
                            'print(json.dumps({"args": sys.argv[1:], "env": dict(os.environ)}))\n')
            pixi.chmod(0o755)
            contamination = {
                'PYTHONPATH': '/old/ros/python:/other/isaac/python', 'PYTHONHOME': '/old/python',
                'LD_LIBRARY_PATH': '/old/ros/lib:/other/isaac/lib',
                'CONDA_PREFIX': '/old/ros', 'CONDA_PREFIX_1': '/another/conda',
                'VIRTUAL_ENV': '/other/venv', 'PIXI_ENVIRONMENT_NAME': 'default',
                'PIXI_PROJECT_ROOT': '/other/project', 'PIXI_PROJECT_MANIFEST': '/other/pixi.toml',
                'PIXI_WORKSPACE_ROOT': '/other/project', 'PIXI_WORKSPACE_MANIFEST': '/other/pixi.toml',
                'AMENT_PREFIX_PATH': '/old/ros', 'ROS_DISTRO': 'humble',
                'CMAKE_PREFIX_PATH': '/old/ros', 'QT_PLUGIN_PATH': '/old/ros/qt',
                'CARB_APP_PATH': '/other/isaac/kit', 'EXP_PATH': '/other/isaac/apps',
                'ISAAC_PATH': '/other/isaac', 'ISAAC_SIM_PATH': '/other/isaac',
                'PXR_PLUGINPATH_NAME': '/other/usd/plugins',
            }
            preserved = {'HOME': os.environ['HOME'], 'DISPLAY': ':41', 'XAUTHORITY': '/tmp/test.xauthority',
                         'CUDA_VISIBLE_DEVICES': '0', 'NVIDIA_VISIBLE_DEVICES': 'all', 'ROS_DOMAIN_ID': '86'}
            for wrapper, mode in [('launch_robot.sh', 'robot'), ('launch_course.sh', 'course'),
                                  ('validate_stage.sh', 'validate')]:
                for acceptance in (None, 'Y'):
                    with self.subTest(wrapper=wrapper, acceptance=acceptance):
                        environment = dict(os.environ, **contamination, **preserved)
                        environment['PATH'] = f'{temporary}:/old/ros/bin:{os.defpath}'
                        environment.pop('OMNI_KIT_ACCEPT_EULA', None)
                        if acceptance is not None:
                            environment['OMNI_KIT_ACCEPT_EULA'] = acceptance
                        arguments = ['--stage', '/tmp/course with spaces.usda', 'literal;$HOME']
                        result = subprocess.run(['/bin/bash', str(HERE / wrapper), *arguments],
                                                cwd=temporary, env=environment, check=True,
                                                text=True, capture_output=True)
                        captured = json.loads(result.stdout)
                        self.assertEqual(captured['args'], [
                            'run', '--manifest-path', str(ROOT / 'pixi.toml'), '-e', 'isaac',
                            '--locked', '-x', 'python', str(HERE / 'runtime.py'), mode, *arguments])
                        for name in contamination:
                            self.assertNotIn(name, captured['env'])
                        for name, value in preserved.items():
                            self.assertEqual(captured['env'][name], value)
                        self.assertEqual(captured['env']['PYTHONNOUSERSITE'], '1')
                        self.assertNotIn('/old/ros', captured['env']['PATH'])
                        self.assertEqual(captured['env'].get('OMNI_KIT_ACCEPT_EULA'), acceptance)

    def test_pixi_failure_does_not_fall_back_to_shared_install(self):
        with tempfile.TemporaryDirectory() as directory:
            temporary = Path(directory)
            pixi = temporary / 'pixi'
            pixi.write_text('#!/bin/bash\nexit 7\n')
            pixi.chmod(0o755)
            standalone = temporary / 'standalone'
            standalone.mkdir()
            (standalone / 'python.sh').write_text('#!/bin/bash\nexit 99\n')
            (standalone / 'python.sh').chmod(0o755)
            result = subprocess.run(['/bin/bash', str(HERE / 'launch_robot.sh'), '--help'],
                                    env=dict(os.environ, PATH=f'{temporary}:{os.defpath}',
                                             ISAAC_SIM_PATH=str(standalone)), capture_output=True)
            self.assertEqual(result.returncode, 7)


class BootstrapTest(unittest.TestCase):
    def test_manifest_keeps_isaac_separate_from_ros(self):
        manifest = tomllib.loads((ROOT / 'pixi.toml').read_text())
        self.assertTrue(manifest['environments']['isaac']['no-default-feature'])
        self.assertEqual(manifest['environments']['isaac']['features'], ['isaac'])
        feature = manifest['feature']['isaac']
        self.assertEqual(feature['platforms'], ['linux-64'])
        self.assertEqual(feature['dependencies']['python'], '3.11.*')
        self.assertEqual(manifest['dependencies']['python'], '3.12.*')
        self.assertEqual(feature['pypi-dependencies']['isaacsim']['version'], '==5.1.0')
        self.assertEqual(set(feature['pypi-dependencies']['isaacsim']['extras']), {'all', 'extscache'})

    def test_shared_interpreter_is_rejected(self):
        with patch.object(runtime, 'PREFIX', Path('/unrelated/isaac/environment')):
            with self.assertRaisesRegex(RuntimeError, 'project'):
                runtime.isolated_isaac_root()

    def test_shared_package_is_rejected_without_importing_it(self):
        with tempfile.TemporaryDirectory() as directory:
            prefix = Path(directory) / 'environment'
            spec = types.SimpleNamespace(origin=str(Path(directory) / 'shared/isaacsim/__init__.py'))
            with (patch.object(runtime, 'PREFIX', prefix), patch.object(sys, 'prefix', str(prefix)),
                  patch.object(sys, 'version_info', (3, 11, 0)),
                  patch.object(runtime.importlib.util, 'find_spec', return_value=spec)):
                with self.assertRaisesRegex(RuntimeError, 'outside the project'):
                    runtime.isolated_isaac_root()

    def test_validator_uses_only_bundled_usd_and_environment_libraries(self):
        with tempfile.TemporaryDirectory() as directory:
            prefix = Path(directory)
            package = prefix / 'lib/python3.11/site-packages/isaacsim'
            usd = package / 'extscache/omni.usd.libs-test'
            for part in (usd / 'pxr', usd / 'bin/deps'):
                part.mkdir(parents=True)
            inherited = {'PYTHONPATH': '/shared/python', 'LD_LIBRARY_PATH': '/ros/lib', 'HOME': '/unchanged'}
            actual = runtime.usd_environment(package, prefix, inherited)
            self.assertEqual(actual['PYTHONPATH'], str(usd))
            self.assertEqual(actual['LD_LIBRARY_PATH'].split(os.pathsep),
                             [str(usd / 'bin'), str(usd / 'bin/deps'), str(prefix / 'lib')])
            self.assertEqual(actual['HOME'], '/unchanged')
            self.assertNotIn('OMNI_KIT_ACCEPT_EULA', actual)
            self.assertEqual(inherited['LD_LIBRARY_PATH'], '/ros/lib')

    def test_missing_usd_cache_fails_instead_of_searching_shared_installs(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(RuntimeError, 'bundled OpenUSD'):
                runtime.usd_environment(Path(directory), Path(directory), {})

    def test_kit_portable_root_is_project_local(self):
        with tempfile.TemporaryDirectory() as directory:
            portable = Path(directory) / 'project_runtime'
            original_home = os.environ.get('HOME')
            with (patch.object(runtime, 'RUNTIME_ROOT', portable),
                  patch.dict(os.environ, {'WARP_CACHE_PATH': '/other/warp/cache'})):
                arguments = runtime.application_arguments()
                self.assertEqual(os.environ['WARP_CACHE_PATH'], str(portable / 'cache/warp'))
            self.assertEqual(arguments[:3], ['--portable', '--portable-root', str(portable)])
            settings = dict(argument[3:].split('=', 1) for argument in arguments[3:])
            self.assertEqual(settings['app/extensions/fsWatcherEnabled'], 'false')
            self.assertEqual(settings['rtx-transient/resourcemanager/localTextureCachePath'],
                             str(portable / 'cache/texturecache'))
            for token in ('omni_global_cache', 'omni_global_data', 'omni_global_logs',
                          'omni_global_config', 'omni_documents', 'shared_documents',
                          'app_documents', 'documents'):
                self.assertTrue(Path(settings[f'app/tokens/{token}']).is_relative_to(portable))
            self.assertTrue(portable.is_dir())
            self.assertEqual(os.environ.get('HOME'), original_home)

    def test_eula_guard_rejects_noninteractive_first_use_without_writing_marker(self):
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {}, clear=True):
            package = Path(directory)
            with self.assertRaisesRegex(RuntimeError, 'pixi run configure-isaac'):
                runtime.require_eula(package, False)
            self.assertEqual(list(package.iterdir()), [])

    def test_eula_recognizes_only_nvidia_explicit_environment_and_local_marker(self):
        with tempfile.TemporaryDirectory() as directory:
            package = Path(directory)
            self.assertFalse(runtime.eula_accepted(package, {}))
            self.assertFalse(runtime.eula_accepted(package, {'OMNI_KIT_ACCEPT_EULA': 'no'}))
            for value in ('YES', 'y', '1'):
                self.assertTrue(runtime.eula_accepted(package, {'OMNI_KIT_ACCEPT_EULA': value}))
            marker = package / 'kit/EULA_ACCEPTED'
            marker.parent.mkdir()
            marker.write_text('yes\n')
            self.assertTrue(runtime.eula_accepted(package, {}))

    def test_configure_invokes_nvidia_prompt_without_setting_acceptance(self):
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {}, clear=True):
            package = Path(directory)
            with (patch.object(sys.stdin, 'isatty', return_value=True),
                  patch.object(runtime.importlib, 'import_module') as load):
                runtime.configure(package)
            load.assert_called_once_with('isaacsim')
            self.assertNotIn('OMNI_KIT_ACCEPT_EULA', os.environ)
            self.assertEqual(list(package.iterdir()), [])


if __name__ == '__main__':
    unittest.main()
