import os
import shlex
import shutil
import subprocess

_gazebo_path = shutil.which('gazebo')
if _gazebo_path is None:
	raise RuntimeError('gazebo is not in your PATH')

_setup_sh_path = os.path.join(os.path.dirname(_gazebo_path), '../share/gazebo/setup.sh')
if os.path.isfile(_setup_sh_path) == False:
	raise RuntimeError('"setup.sh" not found at ' + _setup_sh_path)

def _append_to_default_paths(env_var_name, paths):
	return subprocess.check_output(
		[
			'sh',
			'-c',
			'. {} && echo -n "${}"'.format(
				shlex.quote(_setup_sh_path),
				env_var_name
			)
		],
		env={
			env_var_name:
				os.getenvb(env_var_name.encode('ascii'), b'') +\
				b':' +\
				b':'.join([p.encode('utf-8') for p in paths])
		}
	)

# Create a GAZEBO_PLUGIN_PATH string by appending a list of paths to the default
# ones.
def generate_plugin_path(*paths):
	return _append_to_default_paths('GAZEBO_PLUGIN_PATH', paths)

# Create a GAZEBO_MODEL_PATH string by appending a list of paths to the default
# ones.
def generate_model_path(*paths):
	return _append_to_default_paths('GAZEBO_MODEL_PATH', paths)
