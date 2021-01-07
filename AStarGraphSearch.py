import numpy as np
import random as rr
import copy
import heapq
import pickle
import matplotlib.pyplot as plt
from matplotlib import colors
import os
import sys
import shutil
import multiprocessing
import glob
import IPython


#generate graph with 1s as walls and 0 empty
def randGridMaze(number, width=10, height=10):
	for i in range(0, number+1):
		shape = (height,width)
		Z = np.random.choice([0,1], size=shape, p=[.70,.30])
		plt.figure()
		plt.imshow(Z, cmap="", interpolation='nearest')
		plt.xticks([]), plt.yticks([])
		plt.savefig("pics/randGrid/maze{0:0=2d}.png".format(i))
		np.savetxt("arrs/randGrid/{0:0=2d}.txt".format(i),Z,fmt='%d')
		plt.close('all')

def aStar(width = 10, height = 10):
	for arrFile in os.listdir("arrs/randGrid/"):
		arr = np.loadtxt("arrs/randGrid/"+arrFile).astype(int)
		#Randomly place our target and agent
		goalCol,goalRow = rr.choice(range(0,height)), rr.choice(range(0,width))
		i = 0
		while(arr[goalCol][goalRow]==1):
			if(i==width*height*2):
				#randomly search 2x as many spots we have because I'm lazy and its unlikely
				#just do this because if our grid is all 1s we dont want loop
				
				i=width*height
				break
			goalCol,goalRow = rr.choice(range(0,height)), rr.choice(range(0,width))
			i=i+1
		arr[goalCol][goalRow]=4
		goal = (goalCol, goalRow)
		i=0
		agentCol,agentRow = rr.choice(range(0,height)), rr.choice(range(0,width))
		while(arr[agentCol][agentRow]==1 or arr[agentCol][agentRow]==4):
			if(i==width*height*2):
				#randomly search 2x as many spots we have because I'm lazy and its unlikely
				#just do this because if our grid is all 1s we dont want loop
				i=width*height
				break
			agentCol,agentRow = rr.choice(range(0,height)), rr.choice(range(0,width))
			i=i+1
		arr[agentCol][agentRow]=5
		#Make copy of arr and begin running forward A* big G
		print(arr)
		arrTemp = copy.deepcopy(arr)
		startCol=agentCol
		startRow=agentRow
		start = (startCol, startRow)
		closed=[]
		h=abs(startCol-goalCol)+abs(startRow-goalRow)
		g=0
		f=g+h
		costToReach=0
		parentSpot = start
		startData = (f, g, h, start, 0, start)
		heapq.heappush(closed, startData)
		heap = []
		#check above wall
		checkRow=agentRow
		checkCol=agentCol+1
		if(checkCol<height and checkCol>=0):
			if(arrTemp[checkCol][checkRow]!=1):
				spot = (checkCol, checkRow)
				if arrTemp[checkCol][checkRow]!=4:
						arrTemp[checkCol][checkRow]=2
				h=abs(checkCol-goalCol)+abs(checkRow-goalRow)
				g=abs(checkRow-startRow)+abs(checkCol-startCol)
				f=g+h
				costToReach=1
				parentSpot = start
				hData = (f, g, h, spot, costToReach, parentSpot)
				heapq.heappush(heap, hData)
			#add all neighbors to the stack
		#check left wall
		checkRow=agentRow-1
		checkCol=agentCol
		if(checkRow<width and checkRow>=0):
			if(arrTemp[checkCol][checkRow]!=1):
				spot = (checkCol, checkRow)
				if arrTemp[checkCol][checkRow]!=4:
						arrTemp[checkCol][checkRow]=2
				h=abs(checkCol-goalCol)+abs(checkRow-goalRow)
				g=abs(checkRow-startRow)+abs(checkCol-startCol)
				f=g+h
				costToReach=1
				parentSpot = start
				hData = (f, g, h, spot, costToReach, parentSpot)
				heapq.heappush(heap, hData)
		#check right wall
		checkRow=agentRow+1
		checkCol=agentCol
		if(checkRow<width and checkRow>=0):
			if(arrTemp[checkCol][checkRow]!=1):
				spot = (checkCol, checkRow)
				if arrTemp[checkCol][checkRow]!=4:
						arrTemp[checkCol][checkRow]=2
				h=abs(checkCol-goalCol)+abs(checkRow-goalRow)
				g=abs(checkRow-startRow)+abs(checkCol-startCol)
				f=g+h
				costToReach=1
				parentSpot = start
				hData = (f, g, h, spot, costToReach, parentSpot)
				heapq.heappush(heap, hData)
		#check below wall
		checkRow=agentRow
		checkCol=agentCol-1
		if(checkCol<height and checkCol>=0):
			if(arrTemp[checkCol][checkRow]!=1):
				spot = (checkCol, checkRow)
				if arrTemp[checkCol][checkRow]!=4:
						arrTemp[checkCol][checkRow]=2
				h=abs(checkCol-goalCol)+abs(checkRow-goalRow)
				g=abs(checkRow-startRow)+abs(checkCol-startCol)
				f=g+h
				costToReach=1
				parentSpot = start
				hData = (f, g, h, spot, costToReach, parentSpot)
				heapq.heappush(heap, hData)
		#Begin moving search boy past start node
		while True:
			if not heap:
				print("Cant find")
				break
			tieList=[]
			item = heapq.heappop(heap)
			tieList.append(item)
			if heap:
				#Check if anything else has same fCost
				while heap[0][0]==item[0]:
					item2 = heapq.heappop(heap)
					tieList.append(item2)
					if not heap:
						break
			#find Largest gCost
			if len(tieList)>1:
				for j in range(len(tieList)-1):
					if(tieList[j][1]<tieList[j+1][1]):
						tieList[j], tieList[j+1] = tieList[j+1], tieList[j]
			largest = tieList.pop(0)
			heapq.heappush(closed, largest)
			for tie in tieList:
				heapq.heappush(heap, tie)
			item = largest
			if(arrTemp[item[3][0]][item[3][1]]==2):
				arrTemp[item[3][0]][item[3][1]]=3
			#check if goal
			if(item[3] == goal):
				print("found")
				print(item[3])
				break
			agentCol = item[3][0]
			agentRow = item[3][1]
			checkRow=agentRow
			checkCol=agentCol+1
			checkSpot = (checkCol, checkRow)
			inClose=False
			#check above wall
			for fuck in closed:
				if(fuck[3]==checkSpot):
					inClose=True
			if(checkCol<height and checkCol>=0 and (inClose==False)):
				if(arrTemp[checkCol][checkRow]!=1):
					spot = (checkCol, checkRow)
					if arrTemp[checkCol][checkRow]!=4:
						arrTemp[checkCol][checkRow]=2
					h=abs(checkCol-goalCol)+abs(checkRow-goalRow)
					g=abs(checkRow-startRow)+abs(checkCol-startCol)
					f=g+h
					costToReach = item[4]+1
					parentSpot = (agentCol, agentRow)
					u=0
					for thing1 in heap:
						if(thing1[3]==spot):
							n=0
							for thing2 in heap:
								if(thing1[5]==thing2[3]):
									if(thing2[2]<item[2]):
										heap.pop(u)
										parentSpot = thing1[3]
								n=n+1
						u=u+1
					hData = (f, g, h, spot, costToReach, parentSpot)
					u=0
					for thing1 in heap:
						if(thing1[3]==spot):
							n=0
							for thing2 in closed:
								if(thing1[5]==thing2[3]):
									if(thing2[2]<item[2]):
										parentSpot = thing2[3]
								n=n+1
						u=u+1
					hData = (f, g, h, spot, costToReach, parentSpot)
					heapq.heappush(heap, hData)
				#add all neighbors to the stack
			#check left wall
			checkRow=agentRow-1
			checkCol=agentCol
			checkSpot= (checkCol, checkRow)
			inClose=False
			for fuck in closed:
				if(fuck[3]==checkSpot):
					inClose=True
			if(checkRow<width and checkRow>=0 and (inClose==False)):
				if(arrTemp[checkCol][checkRow]!=1):
					if arrTemp[checkCol][checkRow]!=4:
						arrTemp[checkCol][checkRow]=2
					spot = (checkCol, checkRow)
					h=abs(checkCol-goalCol)+abs(checkRow-goalRow)
					g=abs(checkRow-startRow)+abs(checkCol-startCol)
					f=g+h
					costToReach = item[4]+1
					parentSpot = (agentCol, agentRow)
					hData = (f, g, h, spot, costToReach, parentSpot)
					u=0
					for thing1 in heap:
						if(thing1[3]==spot):
							n=0
							for thing2 in heap:
								if(thing1[5]==thing2[3]):
									if(thing2[2]<item[2]):
										heap.pop(u)
										parentSpot = thing1[3]
								n=n+1
						u=u+1
					hData = (f, g, h, spot, costToReach, parentSpot)
					u=0
					for thing1 in heap:
						if(thing1[3]==spot):
							n=0
							for thing2 in closed:
								if(thing1[5]==thing2[3]):
									if(thing2[2]<item[2]):
										parentSpot = thing2[3]
								n=n+1
						u=u+1
					hData = (f, g, h, spot, costToReach, parentSpot)
					heapq.heappush(heap, hData)
			#check right wall
			checkRow=agentRow+1
			checkCol=agentCol
			checkSpot= (checkCol, checkRow)
			inClose=False
			for fuck in closed:
				if(fuck[3]==checkSpot):
					inClose=True
			if(checkRow<width and checkRow>=0 and (inClose==False)):
				if(arrTemp[checkCol][checkRow]!=1):
					if arrTemp[checkCol][checkRow]!=4:
						arrTemp[checkCol][checkRow]=2
					spot = (checkCol, checkRow)
					h=abs(checkCol-goalCol)+abs(checkRow-goalRow)
					g=abs(checkRow-startRow)+abs(checkCol-startCol)
					f=g+h
					costToReach = item[4]+1
					parentSpot = (agentCol, agentRow)
					hData = (f, g, h, spot, costToReach, parentSpot)
					u=0
					u=0
					for thing1 in heap:
						if(thing1[3]==spot):
							n=0
							for thing2 in heap:
								if(thing1[5]==thing2[3]):
									if(thing2[2]<item[2]):
										heap.pop(u)
										parentSpot = thing1[3]
								n=n+1
						u=u+1
					hData = (f, g, h, spot, costToReach, parentSpot)
					u=0
					for thing1 in heap:
						if(thing1[3]==spot):
							n=0
							for thing2 in closed:
								if(thing1[5]==thing2[3]):
									if(thing2[2]<item[2]):
										parentSpot = thing2[3]
								n=n+1
						u=u+1
					hData = (f, g, h, spot, costToReach, parentSpot)
					heapq.heappush(heap, hData)
			#check below wall
			checkRow=agentRow
			checkCol=agentCol-1
			checkSpot= (checkCol, checkRow)
			inClose=False
			for fuck in closed:
				if(fuck[3]==checkSpot):
					inClose=True
			if(checkCol<height and checkCol>=0 and (inClose==False)):
				if(arrTemp[checkCol][checkRow]!=1):
					if arrTemp[checkCol][checkRow]!=4:
						arrTemp[checkCol][checkRow]=2
					spot = (checkCol, checkRow)
					h=abs(checkCol-goalCol)+abs(checkRow-goalRow)
					g=abs(checkRow-startRow)+abs(checkCol-startCol)
					f=g+h
					costToReach = item[4]+1
					parentSpot = (agentCol, agentRow)
					hData = (f, g, h, spot, costToReach, parentSpot)
					u=0
					for thing1 in heap:
						if(thing1[3]==spot):
							n=0
							for thing2 in heap:
								if(thing1[5]==thing2[3]):
									if(thing2[2]<item[2]):
										heap.pop(u)
										parentSpot = thing1[3]
								n=n+1
						u=u+1
					hData = (f, g, h, spot, costToReach, parentSpot)
					u=0
					for thing1 in heap:
						if(thing1[3]==spot):
							n=0
							for thing2 in closed:
								if(thing1[5]==thing2[3]):
									if(thing2[2]<item[2]):
										parentSpot = thing2[3]
								n=n+1
						u=u+1
					hData = (f, g, h, spot, costToReach, parentSpot)
					heapq.heappush(heap, hData)
		goalSpot = (goalCol, goalRow)
		for thing2 in closed:
			if(thing2[3]==goalSpot):
				while thing2[5]!=thing2[3]:
					agentCol = thing2[5][0]
					agentRow = thing2[5][1]
					agentSpot = (agentCol, agentRow)
					if(arrTemp[agentCol][agentRow]!=5):
						arrTemp[agentCol][agentRow]=6
					for space in closed:
						if(space[3]==agentSpot):
							thing2 = space
							break
		print(arrTemp)
		#start making graph
		plt.figure()
		#0s on our grid are white, 1 is black , 2 is green (opened), 3 is red (closed), orange is goal, yellow is agent
		cmap = colors.ListedColormap(['white', 'black', 'green', 'red', 'orange', 'yellow', 'blue'])
		plt.imshow(arrTemp, cmap, interpolation='nearest')
		plt.xticks([]), plt.yticks([])
		plt.savefig("pics/randGrid/AStar{0:0=2d}.png".format(i))
		np.savetxt("arrs/randGrid/AStar{0:0=2d}.txt".format(i),arrTemp,fmt='%d')
		plt.close('all')

if __name__ == "__main__":
	if os.path.exists("arrs"):
		shutil.rmtree("arrs")
	if os.path.exists("pics"):
		shutil.rmtree("pics")
	if os.path.exists("maze.png"):
		os.remove("maze.png")
	for i in ["", "/randGrid/"]: 
		os.mkdir("pics"+i)
		os.mkdir("arrs"+i)
	### specify the number of grids you want to generate
	n_grids = int(sys.argv[1])
	multiprocessing.freeze_support()
	#num_proc = multiprocessing.cpu_count()
	## for python 3.6 uncomment the line below, and comment the line above
	num_proc = os.cpu_count()
	pool = multiprocessing.Pool(processes = num_proc)

	nn = [i for i in range(n_grids)]
	pool.map(randGridMaze, nn)
	aStar()
    #nn = [i+n_grids for i in nn]
    #pool.map(backTrackerMaze, nn)

	pool.close()
	pool.join()
