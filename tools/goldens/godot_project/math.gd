extends SceneTree
func _initialize():
 var rows=[]
 for a in [Vector3.RIGHT,Vector3.UP,Vector3.BACK,Vector3(1,2,3).normalized(),Vector3(-2,0.3,0.7).normalized()]:
  for b in [a,-a,Vector3(0.5,-0.2,0.8).normalized()]:
   var q=Quaternion(a,b)
   rows.append({"from":[a.x,a.y,a.z],"to":[b.x,b.y,b.z],"quat":[q.x,q.y,q.z,q.w]})
 var file=FileAccess.open(OS.get_cmdline_user_args()[0],FileAccess.WRITE)
 file.store_string(JSON.stringify(rows))
 quit()
