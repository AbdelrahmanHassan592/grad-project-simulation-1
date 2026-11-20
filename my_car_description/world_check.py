"""Read-only SDF compatibility checks; only --fix creates a separate copy."""
import argparse
import os
from pathlib import Path
import re
import struct
import subprocess
import xml.etree.ElementTree as ET

SYSTEMS = {'physics': 'Physics', 'user-commands': 'UserCommands',
           'scene-broadcaster': 'SceneBroadcaster', 'sensors': 'Sensors', 'imu': 'Imu'}

def check_world(filename, fix=False, output_dir=None):
    path = Path(filename).expanduser().resolve()
    warnings, errors = [], []
    if not path.is_file():
        return [f'World does not exist: {path}'], warnings, None
    try:
        tree = ET.parse(path)
    except ET.ParseError as exc:
        return [f'Invalid XML: {exc}'], warnings, None
    worlds = tree.getroot().findall('world')
    if not worlds:
        return ['SDF must contain a world element'], warnings, None
    validation_env=os.environ.copy()
    validation_env['SDF_PATH']=os.pathsep.join(filter(None,[str(path.parent),os.environ.get('GZ_SIM_RESOURCE_PATH',''),os.environ.get('SDF_PATH','')]))
    try:
        result = subprocess.run(['gz', 'sdf', '-k', str(path)], capture_output=True, text=True, timeout=45,env=validation_env)
    except subprocess.TimeoutExpired:
        return ['gz sdf validation timed out after 45 s; check remote assets and network'], warnings, None
    if result.returncode:
        detail=result.stdout+result.stderr
        codes=re.findall(r'Error Code (\d+)',detail)
        remote_includes=[e.text for e in tree.findall('.//include/uri') if e.text and e.text.startswith(('http://','https://'))]
        if remote_includes and codes and set(codes)=={'14'}:
            warnings.append('gz sdf cannot fetch remote include URIs; Gazebo must resolve these Fuel assets with internet: '+', '.join(remote_includes))
        else:
            return ['gz sdf -k failed: ' + detail], warnings, None
    roots = [path.parent]
    roots += [Path(x) for x in os.environ.get('GZ_SIM_RESOURCE_PATH', '').split(os.pathsep) if x]
    roots += [Path.home()/'.gz/fuel', Path.home()/'.gz/models']
    def resolve(uri):
        if uri.startswith(('http://', 'https://')):
            return None
        if uri.startswith('model://'):
            relative = uri[8:]
            for root in roots:
                candidate = root/relative
                if candidate.exists(): return candidate
            # Fuel cache has server/owner/models/name/version directories.
            parts = Path(relative).parts
            cache = Path.home()/'.gz/fuel'
            if cache.exists():
                for model in cache.glob('*/'+'*/models/'+parts[0]+'/*'):
                    candidate = model.joinpath(*parts[1:])
                    if candidate.exists(): return candidate
            return None
        candidate = Path(uri.removeprefix('file://'))
        if not candidate.is_absolute(): candidate = path.parent/candidate
        return candidate if candidate.exists() else None
    changed = False
    for world in worlds:
        for key, cls in SYSTEMS.items():
            plugin = next((p for p in world.findall('plugin') if p.get('name') == 'gz::sim::systems::'+cls or key+'-system' in p.get('filename','')), None)
            if plugin is None:
                warnings.append('Missing system plugin: '+key)
                if fix:
                    plugin = ET.SubElement(world, 'plugin', filename=f'gz-sim-{key}-system', name='gz::sim::systems::'+cls)
                    changed = True
            if key == 'sensors' and plugin is not None and plugin.findtext('render_engine') != 'ogre2':
                warnings.append('Sensors render_engine must be ogre2')
                if fix:
                    engine = plugin.find('render_engine')
                    if engine is None: engine = ET.SubElement(plugin, 'render_engine')
                    engine.text = 'ogre2'
                    changed = True
        models = world.findall('.//model')
        floor = False
        for model in models:
            static = model.findtext('static') in ('true','1')
            name = model.get('name','').lower()
            if model.find('static') is None and any(k in name for k in ('wall','floor','shelf','building','ground')):
                warnings.append(f'Model {name} likely needs <static>true</static>')
            if static:
                if model.find('.//collision/geometry/plane') is not None: floor = True
                for box in model.findall('.//collision/geometry/box/size'):
                    dims = [float(x) for x in box.text.split()]
                    if len(dims)==3 and min(dims[:2])>=1 and dims[2]<=0.5: floor=True
        if not floor: warnings.append('No ground plane or broad static floor found (included models cannot be inspected here)')
        if world.find('.//light') is None: warnings.append('No light found (included models cannot be inspected here)')
        gravity = [float(x) for x in world.findtext('gravity','0 0 -9.8').split()]
        if gravity != [0,0,-9.8]: warnings.append(f'Gravity is {gravity}, expected 0 0 -9.8 m/s^2')
        for step in world.findall('physics/max_step_size'):
            if float(step.text)>0.01: warnings.append('Physics step size exceeds 0.01 s')
    mesh_uris = {e.text.strip() for e in tree.findall('.//mesh/uri') if e.text}
    model_uris = {e.text.strip() for e in tree.findall('.//uri') if e.text and e.text.strip().startswith('model://')}
    for uri in sorted(mesh_uris | model_uris):
        resolved = resolve(uri)
        print(f'URI: {uri} -> {resolved or "UNRESOLVED (remote Fuel requires internet)"}')
        if resolved is None: warnings.append('Cannot resolve URI locally: '+uri)
    for mesh in tree.findall('.//mesh'):
        scale = [float(x) for x in mesh.findtext('scale','1 1 1').split()]
        if max(scale)>=100 or (max(scale)>=1 and any(k in mesh.findtext('uri','').lower() for k in ('millimeter','_mm'))):
            warnings.append('Mesh likely uses millimeters; convert to meters with scale 0.001 0.001 0.001')
        if min(scale)<=0: warnings.append('Mesh scale must be positive')
        resolved = resolve(mesh.findtext('uri',''))
        vertices = []
        if resolved and resolved.is_file():
            if resolved.suffix.lower()=='.obj':
                with resolved.open(errors='ignore') as stream:
                    for line in stream:
                        if line.startswith('v '):
                            try: vertices.append(tuple(float(v) for v in line.split()[1:4]))
                            except ValueError: pass
            elif resolved.suffix.lower()=='.stl':
                with resolved.open('rb') as stream:
                    header=stream.read(84)
                    n=struct.unpack('<I',header[80:84])[0] if len(header)==84 else 0
                    if resolved.stat().st_size==84+50*n:
                        for _ in range(n):
                            values=struct.unpack('<12fH',stream.read(50))
                            vertices.extend([values[3:6],values[6:9],values[9:12]])
                    else:
                        stream.seek(0)
                        for line in stream.read().decode(errors='ignore').splitlines():
                            fields=line.split()
                            if fields and fields[0]=='vertex':
                                try: vertices.append(tuple(float(v) for v in fields[1:4]))
                                except ValueError: pass
            elif resolved.suffix.lower()=='.dae':
                try:
                    for source in ET.parse(resolved).iter():
                        if source.tag.split('}')[-1]=='source' and 'position' in source.get('id','').lower():
                            array=next((e for e in source if e.tag.split('}')[-1]=='float_array'),None)
                            if array is not None and array.text:
                                values=[float(v) for v in array.text.split()]
                                vertices.extend(tuple(values[i:i+3]) for i in range(0,len(values)-2,3))
                except (ET.ParseError,ValueError): pass
        if vertices and all(len(v)==3 for v in vertices) and len(scale)==3:
            extent=max((max(v[i] for v in vertices)-min(v[i] for v in vertices))*scale[i] for i in range(3))
            if extent>100:
                warnings.append(f'Mesh extent is {extent:.2f} m: check millimeter units; a scale of 0.001 converts mm to m')

    for mesh in tree.findall('.//collision/geometry/mesh'):
        resolved = resolve(mesh.findtext('uri',''))
        if resolved and resolved.is_file():
            count = None
            if resolved.suffix.lower()=='.obj':
                count = sum(1 for line in resolved.open(errors='ignore') if line.startswith('f '))
            elif resolved.suffix.lower()=='.stl':
                with resolved.open('rb') as stream:
                    header=stream.read(84)
                if len(header)==84:
                    n=struct.unpack('<I', header[80:84])[0]
                    if resolved.stat().st_size==84+50*n: count=n
                if count is None: count=resolved.read_text(errors='ignore').count('facet normal')
            elif resolved.suffix.lower()=='.dae':
                try:
                    xml=ET.parse(resolved)
                    count=sum(int(e.get('count','0')) for e in xml.iter() if e.tag.split('}')[-1] in ('triangles','polylist','polygons'))
                except ET.ParseError: pass
            if count is not None and count>10000: warnings.append(f'High-poly collision: {resolved} has {count} faces; use primitives or a convex proxy')
            if count is None: warnings.append(f'Collision polygon count unknown for {resolved}')
    output = None
    if fix and not errors:
        source_root=Path(__file__).resolve().parents[1]
        if output_dir:
            destination=Path(output_dir)
        elif (source_root/'package.xml').is_file():
            destination=source_root/'worlds'
        else:
            from ament_index_python.packages import get_package_share_directory
            destination=Path(get_package_share_directory('my_car_description'))/'worlds'
        destination.mkdir(parents=True, exist_ok=True)
        output=destination/(path.stem+'_fixed.sdf')
        if output.resolve()==path: raise ValueError('Refusing to overwrite original')
        # A corrected copy may live away from its assets; preserve local URI targets.
        if output.parent.resolve()!=path.parent.resolve():
            for element in tree.findall('.//uri'):
                if element.text:
                    target=resolve(element.text.strip())
                    if target is not None:element.text=str(target.resolve())
        ET.indent(tree)
        tree.write(output, encoding='utf-8', xml_declaration=True)
        print('Corrected copy: '+str(output))
    return errors, warnings, output

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('world')
    parser.add_argument('--fix', action='store_true')
    args=parser.parse_args()
    errors,warnings,_=check_world(args.world,args.fix)
    for message in warnings: print('WARNING: '+message)
    for message in errors: print('FATAL: '+message)
    if not warnings and not errors: print('PASS: valid, compatible world')
    raise SystemExit(1 if errors else 0)
