with open("./run6/OverHeadPath.txt",'r') as fr:
	with open("./run6/OverHeadPathNew.txt",'w') as fw:
		for line in fr:
			data=line.split()
			dataNew=[data[0],data[3],data[2],data[1]]
			fw.write("  ".join(dataNew))
			fw.write("\n")

