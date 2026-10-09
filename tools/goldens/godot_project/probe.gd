extends SceneTree
func _initialize():
 for cls in ["SpringBoneSimulator3D","TwoBoneIK3D","FABRIK3D","CCDIK3D","JacobianIK3D","SplineIK3D","SpringBoneCollisionSphere3D","JointLimitationCone3D"]:
  var obj = ClassDB.instantiate(cls)
  if obj.has_method("set_setting_count"): obj.set_setting_count(1)
  print("CLASS ",cls)
  for prop in obj.get_property_list():
   if prop.name.begins_with("settings/") or prop.name in ["max_iterations","min_distance","angular_delta_limit","deterministic","radius","height","inside","angle","path","tilt_enabled"]: print(prop.name,"=",obj.get(prop.name))
  if obj is Node: obj.free()
 quit()
