extends SceneTree
var sk: Skeleton3D
var modifier: SkeletonModifier3D
var target: Node3D
var pole: Node3D
var path: Path3D
var scenario: String
var frame := 0
var ready := false
var output: Dictionary
var inputs: Array
var captured: Array
func vec(v: Vector3): return [v.x,v.y,v.z]
func quat(q: Quaternion): return [q.x,q.y,q.z,q.w]
func pose(t: Transform3D): return vec(t.origin)+quat(t.basis.get_rotation_quaternion())+vec(t.basis.get_scale())
func _initialize():
 scenario = OS.get_cmdline_user_args()[0]
 call_deferred("setup")
func setup():
 var rig = Node3D.new()
 root.add_child(rig)
 sk = Skeleton3D.new()
 sk.name = "Skeleton"
 sk.modifier_callback_mode_process = 2
 rig.add_child(sk)
 var count = 5 if scenario.begins_with("spring") else (3 if scenario.begins_with("two") else 6)
 for i in count:
  sk.add_bone("bone%d" % i)
  if i > 0: sk.set_bone_parent(i, i-1)
  sk.set_bone_rest(i, Transform3D(Basis.IDENTITY, Vector3.ZERO if i==0 else Vector3(0,0.5,0)))
  sk.reset_bone_pose(i)
 target = Node3D.new()
 target.name = "Target"
 rig.add_child(target)
 pole = Node3D.new()
 pole.name = "Pole"
 rig.add_child(pole)
 var cls = "SpringBoneSimulator3D"
 if scenario.begins_with("two"): cls = "TwoBoneIK3D"
 elif scenario.begins_with("fabrik"): cls = "FABRIK3D"
 elif scenario.begins_with("ccd"): cls = "CCDIK3D"
 elif scenario.begins_with("jacobian"): cls = "JacobianIK3D"
 elif scenario.begins_with("spline"): cls = "SplineIK3D"
 modifier = ClassDB.instantiate(cls)
 sk.add_child(modifier)
 modifier.set("setting_count", 1)
 modifier.set("settings/0/root_bone",0)
 modifier.set("settings/0/end_bone",count-1)
 if scenario.begins_with("spring"):
  modifier.set("settings/0/stiffness/value", 4.0 if scenario=="spring_stiff" else 1.0)
  modifier.set("settings/0/drag/value",0.1 if scenario=="spring_loose" else 0.4)
  if scenario=="spring_gravity": modifier.set("settings/0/gravity/value",0.5)
  if scenario=="spring_hinge": modifier.set("settings/0/rotation_axis",2)
  if scenario=="spring_center_bone":
   modifier.set("settings/0/center_from",2)
   modifier.set("settings/0/center_bone",0)
  if scenario=="spring_external": modifier.set("external_force",Vector3(0.7,0.1,0.3))
  if scenario in ["spring_sphere","spring_capsule","spring_inside_sphere","spring_plane"]:
   var collider_cls = "SpringBoneCollisionSphere3D"
   if scenario=="spring_capsule": collider_cls="SpringBoneCollisionCapsule3D"
   elif scenario=="spring_plane": collider_cls="SpringBoneCollisionPlane3D"
   var collider = ClassDB.instantiate(collider_cls)
   modifier.add_child(collider)
   collider.position = Vector3(0.2,1.1,0.05)
   if scenario=="spring_plane":
    collider.position=Vector3(0,-0.1,0)
    collider.rotation.z = 0.45
   else:
    collider.set("radius",0.35 if scenario!="spring_inside_sphere" else 1.6)
    if scenario=="spring_capsule": collider.set("height",1.1)
    collider.set("inside",scenario=="spring_inside_sphere")
 elif scenario.begins_with("two"):
  modifier.set("settings/0/middle_bone",1)
  modifier.set("settings/0/target_node",modifier.get_path_to(target))
  modifier.set("settings/0/pole_node",modifier.get_path_to(pole))
  if scenario=="two_virtual":
   modifier.set("settings/0/use_virtual_end",true)
   modifier.set("settings/0/extend_end_bone",true)
   modifier.set("settings/0/end_bone/direction",2)
   modifier.set("settings/0/end_bone/length",0.5)
 elif scenario.begins_with("spline"):
  path = Path3D.new()
  path.name="Path"
  rig.add_child(path)
  var curve = Curve3D.new()
  curve.bake_interval=0.05
  for i in 16:
   var a = float(i)*0.35
   var p = Vector3(sin(a)*0.45,float(i)*0.2,cos(a)*0.45-0.45)
   var tangent=Vector3(cos(a)*0.45*0.35,0.2,-sin(a)*0.45*0.35)/3.0
   curve.add_point(p,-tangent,tangent)
   curve.set_point_tilt(i,float(i)*0.1)
  path.curve=curve
  modifier.set("settings/0/path_3d",modifier.get_path_to(path))
  modifier.set("settings/0/tilt_enabled",scenario=="spline_tilt")
 else:
  modifier.set("settings/0/target_node",modifier.get_path_to(target))
  modifier.set("deterministic",not scenario.ends_with("warm"))
  if scenario.ends_with("limits"):
   for j in range(2,5):
    var limit=JointLimitationCone3D.new()
    limit.angle=0.45
    modifier.set("settings/0/joints/%d/limitation"%j,limit)
 modifier.active=false
 sk.advance(0.0)
 sk.notification(50)
 modifier.active=true
 modifier.modification_processed.connect(capture)
 output={"scenario":scenario,"godot_commit":"ed1daf0bf001b61586d9930840f2f1394092c079","rest":[],"parents":[],"frames":[]}
 for i in count:
  output.rest.append(pose(sk.get_bone_rest(i)))
  output.parents.append(null if i==0 else i-1)
 if path:
  output.points=[]
  output.tilts=[]
  for p in path.curve.get_baked_points(): output.points.append(vec(p))
  for t in path.curve.get_baked_tilts(): output.tilts.append(t)
 call_deferred("begin")
func begin():
 sk.position=Vector3(0,0,0.08)
 ready=true
func capture():
 captured=[]
 for i in sk.get_bone_count():
  var t=sk.global_transform*sk.get_bone_global_pose(i)
  captured.append(vec(t.origin)+quat(t.basis.get_rotation_quaternion()))
func _process(delta):
 if not ready: return false
 var t=float(frame)/60.0
 sk.position=Vector3(sin(t*1.7)*0.22,0,cos(t*0.9)*0.08)
 sk.rotation.z=sin(t*0.8)*0.12
 target.position=sk.position+Vector3(sin(t*1.4)*1.25,0.7+cos(t*0.7)*0.55,cos(t)*0.45)
 pole.position=sk.position+Vector3(0.5,0.6,1.0)
 inputs=[]
 for i in sk.get_bone_count():
  sk.reset_bone_pose(i)
  inputs.append(pose(sk.get_bone_pose(i)))
 captured=[]
 sk.advance(delta)
 sk.notification(50)
 assert(captured.size()==sk.get_bone_count(),"native modifier pipeline did not run")
 output.frames.append({"delta":delta,"root":pose(sk.global_transform),"input":inputs,"output":captured,"target":vec(sk.global_transform.affine_inverse()*target.global_position),"pole":vec(sk.global_transform.affine_inverse()*pole.global_position)})
 frame+=1
 if frame==300:
  var file=FileAccess.open(OS.get_cmdline_user_args()[1],FileAccess.WRITE)
  file.store_string(JSON.stringify(output))
  quit()
 return false
