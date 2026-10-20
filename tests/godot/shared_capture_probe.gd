extends SceneTree
var checks:=0
var failures:=0

func check(ok: bool,label: String) -> void:
	checks+=1
	if not ok:
		failures+=1
		push_error(label)

func write_header(name: String,sequence: int,width: int=16) -> void:
	var file:=FileAccess.open("/dev/shm/"+name,FileAccess.READ_WRITE)
	file.seek(8)
	file.store_64(sequence)
	file.seek(24)
	file.store_32(width)
	file.flush()
	file.close()

func _initialize() -> void:
	call_deferred("run")

func encoded_frame(source: Node) -> Dictionary:
	check(source.request_rgbd_encode(3840,95),"native encoder accepts a fresh snapshot")
	for _attempt in 100:
		var frame: Dictionary=source.take_rgbd_encoded_frame()
		if not frame.is_empty(): return frame
		await create_timer(0.01).timeout
	return {}

func decoded_depth(frame: Dictionary) -> PackedByteArray:
	var count:=int(frame.width)*int(frame.height)
	var raw: PackedByteArray=frame.depth_compressed.decompress(count*2,FileAccess.COMPRESSION_DEFLATE)
	if frame.depth_encoding=="u16_mm_deflate": return raw
	var decoded:=PackedByteArray()
	decoded.resize(count*2)
	for i in count:
		var zig:=int(raw[i])+(int(raw[count+i])<<8)
		var delta: int=(zig>>1)^(-(zig&1))
		var above:=decoded.decode_u16((i-int(frame.width))*2) if i>=int(frame.width) else 0
		decoded.encode_u16(i*2,(above+delta)&65535)
	return decoded

func run() -> void:
	var args:=OS.get_cmdline_user_args()
	var input: String=args[0]
	var output: String=args[1]
	var reader=ClassDB.instantiate("RealSenseSharedMemoryReader")
	check(not reader.open("../invalid"),"mapping names cannot escape the shared namespace")
	check(reader.open(input) and reader.poll(),"read a POSIX frame without opening a camera")
	check(reader.get_width()==16 and reader.get_height()==16 and reader.get_stride()==1,"native dimensions retained")
	check(reader.get_intrinsics()==Vector4(20,21,8,8),"native intrinsics retained")
	check(is_equal_approx(reader.get_depth_image().get_pixel(5,7).r,1.117),"depth is copied exactly")
	check(reader.get_color_image().get_pixel(5,7).is_equal_approx(Color8(5,7,12,255)),"original RGBA retained")
	check(reader.get_capture_fps()==60.0,"source cadence retained separately from preview cadence")
	check(not reader.poll(),"unchanged acquisition is never counted twice")
	write_header(input,3)
	check(not reader.poll(),"in-progress snapshot rejected")
	write_header(input,4,0)
	check(not reader.poll(),"malformed dimensions rejected")
	write_header(input,6)
	check(reader.poll(),"complete snapshot resumes after rejected writes")
	reader.close()
	check(reader.open(input) and reader.poll(),"reopening resets the sequence cache")
	var copier=ClassDB.instantiate("RealSenseSharedMemoryPointCloud")
	copier.set_capture_only(true)
	copier.set_shared_memory_name(input)
	copier.set_publish_shared_memory_name(output)
	root.process_thread_group=Node.PROCESS_THREAD_GROUP_MAIN_THREAD
	root.process_thread_group_order=1
	copier.process_thread_group=Node.PROCESS_THREAD_GROUP_SUB_THREAD
	copier.process_thread_group_order=0
	root.add_child(copier)
	for _frame in 4: await process_frame
	var preview=ClassDB.instantiate("RealSenseSharedMemoryReader")
	check(preview.open(output) and preview.poll(),"independent preview receives a published snapshot")
	check(preview.get_depth_image().get_data()==reader.get_depth_image().get_data(),"shared publication preserves every depth byte")
	check(preview.get_color_image().get_data()==reader.get_color_image().get_data(),"shared publication preserves every RGB byte")
	var raw_grid: Dictionary=copier.get_depth_u16_frame(3840)
	check(raw_grid.depth_mm.decode_u16((8*16+8)*2)==0,"native-sized synchronous depth never fills a measured hole")
	var fast:=await encoded_frame(copier)
	check(not fast.is_empty() and decoded_depth(fast)==raw_grid.depth_mm,"fast encoding preserves every native millimetre and measured hole")
	copier.set_color_match_enabled(true)
	copier.set_color_gain(Vector3.ONE)
	copier.set_color_bias(Vector3.ZERO)
	write_header(input,8)
	for _frame in 4: await process_frame
	var reference:=await encoded_frame(copier)
	check(fast.color_jpeg==reference.color_jpeg,"direct JPEG scanlines exactly match the original identity resampler")
	var previous: int=copier.get_frame_sequence()
	write_header(input,2)
	for _frame in 4: await process_frame
	check(copier.get_frame_sequence()>previous,"encoder sequence remains monotonic after source restart")
	preview.close()
	reader.close()
	copier.free()
	print("SHARED_CAPTURE checks=%d failures=%d" % [checks,failures])
	quit(0 if failures==0 else 1)
