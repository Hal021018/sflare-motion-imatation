"""Explicit code-only deployment. Never starts/stops nodes or calls robot APIs."""
import argparse
import datetime
import hashlib
import os
from pathlib import Path
import shlex
import tarfile
import tempfile
import paramiko

ROOT = Path(__file__).resolve().parents[1]
BASE = '/home/aidlux/RobotGK'

def run(client, command):
    _, out, err = client.exec_command(command, timeout=600)
    print(out.read().decode(errors='replace'))
    print(err.read().decode(errors='replace'))
    code = out.channel.recv_exit_status()
    if code:
        raise RuntimeError(f'remote exit={code}: {command}')

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('host')
    parser.add_argument('--deploy', action='store_true')
    parser.add_argument('--verify', action='store_true')
    args = parser.parse_args()
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    client.set_missing_host_key_policy(paramiko.WarningPolicy())
    client.connect(args.host, username='aidlux', password=os.environ['DEPLOY_SSH_PASSWORD'],
                   timeout=15, auth_timeout=15, banner_timeout=15)
    try:
        run(client, 'hostname; ls -ld /home/aidlux/RobotGK; '
            'ls -ld /home/aidlux/RobotGK/sflare_action_service /home/aidlux/RobotGK/sflare_game_ros '
            '/usr/local/lib/PallasSDK /opt/ros/humble /home/aidlux/Haier_robot_ws/install 2>/dev/null || true; '
            'ps -eo pid,args | grep -E "[d]ual_arm_fist_stretch|[g]ame_node|[c]olcon" || true; '
            'ls -l /usr/local/lib/PallasSDK/libPallasSDK.so 2>/dev/null || true')
        if args.verify:
            sftp = client.open_sftp()
            try:
                files = ['src/sflare_action_service/'+name for name in (
                    'movej_coordinator.py','motion_backend.py','action_program.py',
                    'dual_arm_fist_stretch.py','start.sh')]
                files += ['src/sflare_action_service/actions/'+name+'.json' for name in
                          ('like','handshake','handover','diagonal_point','raise_head','doubt')]
                files += ['src/sflare_game_controller/sflare_game_controller/imitate_node.py']
                for name in files:
                    remote = BASE+'/'+name.removeprefix('src/') if name.startswith('src/sflare_action_service/') else BASE+'/sflare_game_ros/'+name
                    with sftp.open(remote, 'rb') as stream:
                        actual = hashlib.sha256(stream.read()).hexdigest()
                    if actual != hashlib.sha256((ROOT/name).read_bytes()).hexdigest():
                        raise RuntimeError('Hash mismatch: '+remote)
                print(f'Verified {len(files)} key deployed files against local source.')
            finally:
                sftp.close()
        if not args.deploy:
            return
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        stage = BASE+'/motion_code_stage_'+stamp
        backup = BASE+'/before_motion_update_'+stamp+'.tar.gz'
        with tempfile.TemporaryDirectory(prefix='motion_deploy_') as temp:
            archive = Path(temp)/'payload.tar.gz'
            with tarfile.open(archive, 'w:gz') as tar:
                for folder in ('src/sflare_action_service', 'src/sflare_game_controller',
                               'src/sflare_game_interfaces', 'docs'):
                    for path in sorted((ROOT/folder).rglob('*')):
                        if not path.is_file() or any(p in {'__pycache__','.pytest_cache','.git','traces'} for p in path.parts):
                            continue
                        if path.suffix in {'.pyc', '.so'}:
                            continue
                        tar.add(path, arcname=path.relative_to(ROOT).as_posix())
                for name in ('API.md','HANDOVER.md','README.md'):
                    tar.add(ROOT/name, arcname=name)
            run(client, 'mkdir -p '+BASE+' && test -d '+BASE+' && test ! -e '+stage+' && mkdir '+stage)
            sftp = client.open_sftp()
            try:
                sftp.put(str(archive), stage+'/payload.tar.gz')
            finally:
                sftp.close()
            digest = hashlib.sha256(archive.read_bytes()).hexdigest()
            run(client, f'test "$(sha256sum {stage}/payload.tar.gz | cut -d\' \' -f1)" = {digest} && '
                f'tar -xzf {stage}/payload.tar.gz -C {stage}')
            # Backup before any overwrite; archive only explicitly named project dirs.
            run(client, f'cd {BASE} && items=""; '
                'for item in sflare_action_service sflare_game_ros; do '
                'if [ -e "$item" ]; then items="$items $item"; fi; done; '
                f'if [ -n "$items" ]; then tar -czf {backup} $items; '
                f'else tar -czf {backup} --files-from /dev/null; fi')
            run(client, f'mkdir -p {BASE}/sflare_action_service {BASE}/sflare_game_ros/src && '
                f'cp -a {stage}/src/sflare_action_service/. {BASE}/sflare_action_service/ && '
                f'cp -a {stage}/src/sflare_game_controller {stage}/src/sflare_game_interfaces {BASE}/sflare_game_ros/src/ && '
                f'mkdir -p {BASE}/sflare_game_ros/docs && '
                f'cp -a {stage}/docs/. {BASE}/sflare_game_ros/docs/ && '
                f'cp -a {stage}/API.md {stage}/HANDOVER.md {stage}/README.md {BASE}/sflare_game_ros/ && '
                f'python3 -m compileall -q {BASE}/sflare_action_service {BASE}/sflare_game_ros/src')
            command = ('set -e; source /opt/ros/humble/setup.bash; '
                       'source /home/aidlux/Haier_robot_ws/install/setup.bash; '
                       f'cd {BASE}/sflare_game_ros; '
                       'colcon build --symlink-install --packages-select sflare_game_interfaces sflare_game_controller')
            run(client, 'bash -lc '+shlex.quote(command))
            print('BACKUP='+backup)
            print('STAGING='+stage)
            print('Code copied and built. No node restart or robot commands issued.')
    finally:
        client.close()

if __name__ == '__main__':
    main()
