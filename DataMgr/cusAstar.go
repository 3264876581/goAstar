package DataMgr

import (
	"container/heap"
	"strconv"

	//"container/heap"
	"fmt"
	"math"
	"time"
)

type Node struct {
	X, Y     int
	nodeType int //0:road,1:wall
	f, g, h  float64
	//
	father *Node
	//
	open       bool
	closed     bool
	openIndex  int
	closeIndex int
}

type DirNode struct {
	dirX, dirY int
}

func InitNode(x, y, nodeType int) *Node {
	return &Node{
		X:          x,
		Y:          y,
		nodeType:   nodeType, //0:road,1:wall
		f:          0,
		g:          0,
		h:          0,
		father:     nil,
		open:       false,
		closed:     false,
		openIndex:  -1,
		closeIndex: -1,
	}
}

func InitDirNode(dirx, diry int) *DirNode {
	return &DirNode{
		dirX: dirx,
		dirY: diry,
	}
}

// PriorityQueue 实现一个最小堆
type PriorityQueue []*Node

func (pq PriorityQueue) Len() int            { return len(pq) }
func (pq PriorityQueue) Less(i, j int) bool  { return pq[i].f < pq[j].f }
func (pq PriorityQueue) Swap(i, j int)       { pq[i], pq[j] = pq[j], pq[i] }
func (pq *PriorityQueue) Push(x interface{}) { *pq = append(*pq, x.(*Node)) }
func (pq *PriorityQueue) Pop() interface{} {
	old := *pq
	n := len(old)
	item := old[n-1]
	*pq = old[:n-1]
	return item
}

type MapManager struct {
	mapData    [][]*Node
	rows       int
	cols       int
	startNodeX int
	startNodeY int
	openList   PriorityQueue
	closeList  []*Node
	//heap       *Heap
	//
	closePathIndex                       int
	printPathList                        [][]string
	openListChangeFlag                   bool
	pathFindFlag                         bool
	readySmoothPathList                  []*Node
	dirPathList                          []*DirNode
	nowDirX                              int
	nowDirY                              int
	temporaryImportantIndexOfUselessNode []*Node
	afterUseLessPathList                 []*Node
	FinalPathList                        []*Node
	importantFinalInflectionIndex        []int
	allFinalInflectionIndex              []int
	SmoothValType                        *smoothVal
	printTime                            *printTimeToken
}

type smoothVal struct {
	SmoothFinalIndex []int
	startIndex       int
	midIndex         int
	endIndex         int
	//obstacleJudgeList                   []*Node
	importantCombinationsFinalIndexMap  map[int][][]int     //key:[Combinations],value:[[][][]]
	allCombinationsFinalIndexMap        map[int][][]int     //key:[Combinations],value:[[][][]]
	successAllObstacleFinalIndexHValMap map[int]map[int]int //key:[startFinalIndex],value:[endFinalIndex],son value:H(Manhattan cost)
	temporaryJudgeList                  []*Node
	allPass                             bool
	k                                   float64
	b                                   float64
	lineX                               float64
	lineY                               float64
	lowerInt                            int
	upperInt                            int
	disLow                              float64
	disUpper                            float64
	calculateVal                        int
}

func reverseSlice(s []*Node) {
	for i, j := 0, len(s)-1; i < j; i, j = i+1, j-1 {
		s[i], s[j] = s[j], s[i]
	}
}

func (mapManager *MapManager) SetObstacle(xIndex, yIndex int) {
	mapManager.mapData[xIndex][yIndex].nodeType = 1 //Obstacle
	mapManager.printPathList[xIndex][yIndex] = "!"
}

func (mapManager *MapManager) SetRoad(xIndex, yIndex int) {
	mapManager.mapData[xIndex][yIndex].nodeType = 0 //road
	mapManager.printPathList[xIndex][yIndex] = "."
}

// NewMapManager
// @width :rows
// @height :cols
// return :*MapManager
func NewMapManager(width, height int) *MapManager {
	time1 := time.Now().UnixMilli()
	rows, cols := height, width // 定义行数和列数
	twoDSlice := make([][]*Node, rows)
	printList := make([][]string, rows)
	// 初始化二维切片的每一行
	for i := range twoDSlice {
		twoDSlice[i] = make([]*Node, cols)
		printList[i] = make([]string, cols)
	}
	for i := 0; i < rows; i++ {
		for j := 0; j < cols; j++ {
			twoDSlice[i][j] = InitNode(i, j, 0)
			printList[i][j] = "."
		}
	}
	defer func() {
		time2 := time.Now().UnixMilli()
		fmt.Println("\nNewMapManager Start Time:", time1)
		fmt.Println("NewMapManager End Time:", time2)
		fmt.Println("NewMapManager Time Taken:", time2-time1, "milliseconds")
	}()
	//
	return &MapManager{
		mapData:    twoDSlice,
		rows:       height,
		cols:       width,
		startNodeX: 0,
		startNodeY: 0,
		//open close
		openList:  make(PriorityQueue, 0, 1024*1024),
		closeList: make([]*Node, 0, 1024*4),
		//heap: &Heap{},
		//path and smooth path
		closePathIndex:                       -1,
		printPathList:                        printList,
		openListChangeFlag:                   false,
		pathFindFlag:                         false,
		readySmoothPathList:                  make([]*Node, 0, 1024*4),
		dirPathList:                          make([]*DirNode, 0, 1024*4),
		nowDirX:                              0,
		nowDirY:                              0,
		temporaryImportantIndexOfUselessNode: make([]*Node, 0, 32),
		afterUseLessPathList:                 make([]*Node, 0, 1024),
		FinalPathList:                        make([]*Node, 0, 64),
		importantFinalInflectionIndex:        make([]int, 0, 64),
		allFinalInflectionIndex:              make([]int, 0, 64),
		SmoothValType: &smoothVal{
			//obstacleJudgeList:                   make([]*Node, 0, 2),
			importantCombinationsFinalIndexMap:  make(map[int][][]int, 64),     //key:[Combinations],value:[[][][]]
			allCombinationsFinalIndexMap:        make(map[int][][]int, 64),     //key:[Combinations],value:[[][][]]
			successAllObstacleFinalIndexHValMap: make(map[int]map[int]int, 64), //key:[startFinalIndex],value:[endFinalIndex],son value:H
			SmoothFinalIndex:                    make([]int, 0, 1024*4),
			temporaryJudgeList:                  make([]*Node, 0, 1024*4),
			allPass:                             false},
		printTime: &printTimeToken{},
	}
}

var rangeOffset = [][]int{
	{0, 1},   // up
	{0, -1},  // down
	{-1, 0},  // left
	{1, 0},   // right
	{-1, 1},  //left up
	{1, 1},   //right up
	{-1, -1}, //left down
	{1, -1},  //right down
	//
	//{-1, -2}, //left up
	//{-2, -1}, //right up
	//{-2, 1},  //left down
	//{-1, 2},  //right down
	////
	//{1, 2},  //left up
	//{2, 1},  //right up
	//{2, -1}, //left down
	//{1, -2}, //right down
}

func (mapManager *MapManager) printMap() {
	for i := 0; i < mapManager.rows; i++ {
		fmt.Println()
		for j := 0; j < mapManager.cols; j++ {
			fmt.Print(mapManager.printPathList[i][j])
		}
	}
	fmt.Println()
}

// PathFind true:Success Find a road
// result: SmoothFinalIndex(some index of FinalPathList slice)
// node: FinalPathList(Inflection node)
func (mapManager *MapManager) PathFind(x1, y1, x2, y2 int, printResultFlag, printMapFlag, printTimeTokenFlag bool) bool {
	mapManager.printTime.StartTime = time.Now().UnixMicro()
	//boundary status
	if x1 < 0 || y1 < 0 || x2 < 0 || y2 < 0 || x1 >= mapManager.rows || y1 >= mapManager.cols || x2 >= mapManager.rows || y2 >= mapManager.cols {
		fmt.Println("PathFind Failed:start Index or end Index is out of map range!")
		return false
	}
	if x1 == x2 && y1 == y2 {
		fmt.Println("PathFind Failed:start Index and end Index is same!")
		return false
	}
	//judge wall
	if mapManager.mapData[x1][y1].nodeType == 1 || mapManager.mapData[x2][y2].nodeType == 1 {
		fmt.Println("PathFind Failed:start Index or end Index is Obstacle!")
		return false
	}
	//first judge Obstacle of startIndex and endIndex
	if mapManager.obstacleJudge(mapManager.mapData[x1][y1], mapManager.mapData[x2][y2]) {
		//
		mapManager.FinalPathList = mapManager.FinalPathList[:0]
		mapManager.SmoothValType.SmoothFinalIndex = mapManager.SmoothValType.SmoothFinalIndex[:0]
		mapManager.FinalPathList = append(mapManager.FinalPathList, mapManager.mapData[x1][y1], mapManager.mapData[x2][y2])
		mapManager.SmoothValType.SmoothFinalIndex = append(mapManager.SmoothValType.SmoothFinalIndex, 0, 1)
		mapManager.printTime.EndTime = time.Now().UnixMicro()
		//printMap
		if printMapFlag {
			for index, val := range mapManager.FinalPathList {
				mapManager.printPathList[val.X][val.Y] = strconv.Itoa(index)
			}
			//printMap
			mapManager.printMap()
		}
		//printResult
		if printResultFlag {
			mapManager.printResult()
		}
		//printTime
		if printTimeTokenFlag {
			fmt.Printf("%-25s %d μs\n", "pathFind Token(same line):", mapManager.printTime.EndTime-mapManager.printTime.StartTime)
		}
		//success obstacleJudge return true
		return true
	}
	//resetMapData
	mapManager.resetMapData(x1, y1)
	mapManager.printTime.ResetTimeCost = time.Now().UnixMicro() - mapManager.printTime.StartTime
	//pathFind
	mapManager.pathFind(x1, y1, x2, y2)
	//pathFind success
	if mapManager.pathFindFlag {
		//SmoothPath
		mapManager.smoothPath()
		mapManager.printTime.PathFindCost = time.Now().UnixMicro() - mapManager.printTime.StartTime
		//printMap
		if printMapFlag {
			for index, val := range mapManager.FinalPathList {
				mapManager.printPathList[val.X][val.Y] = strconv.Itoa(index)
			}
			mapManager.printMap()
		}
		//printResult
		if printResultFlag {
			mapManager.printResult()
		}
		//printTime
		if printTimeTokenFlag {
			fmt.Printf("%-25s %d μs\n", "ResetTimeCost Taken:", mapManager.printTime.ResetTimeCost)
			fmt.Printf("%-25s %d μs\n", "pathFind Taken:", mapManager.printTime.PathFindCost)
		}
	}
	//
	return mapManager.pathFindFlag
}

func (mapManager *MapManager) printResult() {
	fmt.Println("\ncheck FinalPathList:")
	for index, val := range mapManager.FinalPathList {
		fmt.Println(index, " -> ", " x: ", val.X, " y: ", val.Y, " nodeType: ", val.nodeType)
	}
	fmt.Println("check SmoothFinalIndex:")
	for _, val := range mapManager.SmoothValType.SmoothFinalIndex {
		fmt.Println(" index ", val)
	}
}

//x1:start index x2:end index
func (mapManager *MapManager) resetMapData(x1, y1 int) {
	//reset mapData and printPathList in openList and closeList
	i := 0
	for i = 0; i < len(mapManager.openList); i++ {
		mapManager.resetNodeFromMapData(mapManager.openList[i].X, mapManager.openList[i].Y)
	}
	for i = 0; i < len(mapManager.closeList); i++ {
		mapManager.resetNodeFromMapData(mapManager.closeList[i].X, mapManager.closeList[i].Y)
	}
	//reset mapData and printPathList in start Node
	mapManager.resetNodeFromMapData(mapManager.startNodeX, mapManager.startNodeY)
	//reset startNode index
	mapManager.startNodeX = x1
	mapManager.startNodeY = y1
	//reset openList and closeList
	mapManager.openList = mapManager.openList[:0]
	mapManager.closeList = mapManager.closeList[:0]
	//
	mapManager.closePathIndex = -1
	//reset openListChangeFlag
	mapManager.openListChangeFlag = false
	//reset readySmoothPathList
	mapManager.pathFindFlag = false
	mapManager.readySmoothPathList = mapManager.readySmoothPathList[:0]
	//dir pathList,temporaryImportantIndexOfUselessNode , fina PathList
	mapManager.dirPathList = mapManager.dirPathList[:0]
	mapManager.temporaryImportantIndexOfUselessNode = mapManager.temporaryImportantIndexOfUselessNode[:0]
	mapManager.afterUseLessPathList = mapManager.afterUseLessPathList[:0]
	mapManager.FinalPathList = mapManager.FinalPathList[:0]
	//important inflectionIndex and unimportant inflectionIndex
	mapManager.importantFinalInflectionIndex = mapManager.importantFinalInflectionIndex[:0]
	mapManager.allFinalInflectionIndex = mapManager.allFinalInflectionIndex[:0]
	//---------------------------------------------------------------------------SmoothValType
	//
	//mapManager.SmoothValType.obstacleJudgeList = mapManager.SmoothValType.obstacleJudgeList[:0]
	clear(mapManager.SmoothValType.importantCombinationsFinalIndexMap)
	clear(mapManager.SmoothValType.allCombinationsFinalIndexMap)
	clear(mapManager.SmoothValType.successAllObstacleFinalIndexHValMap)
	mapManager.SmoothValType.SmoothFinalIndex = mapManager.SmoothValType.SmoothFinalIndex[:0]
	mapManager.SmoothValType.startIndex = 0
	mapManager.SmoothValType.midIndex = 0
	mapManager.SmoothValType.endIndex = 0
	mapManager.SmoothValType.allPass = false
	mapManager.SmoothValType.k = 0
	mapManager.SmoothValType.b = 0
	mapManager.SmoothValType.lineX = 0
	mapManager.SmoothValType.lineY = 0
	mapManager.SmoothValType.lowerInt = 0
	mapManager.SmoothValType.upperInt = 0
	mapManager.SmoothValType.disLow = 0
	mapManager.SmoothValType.disUpper = 0
	mapManager.SmoothValType.calculateVal = 0
}

// reset Node From MapData
func (mapManager *MapManager) resetNodeFromMapData(x, y int) {
	mapManager.mapData[x][y].f = 0
	mapManager.mapData[x][y].g = 0
	mapManager.mapData[x][y].h = 0
	mapManager.mapData[x][y].father = nil
	mapManager.mapData[x][y].open = false
	mapManager.mapData[x][y].closed = false
	mapManager.mapData[x][y].openIndex = -1
	mapManager.mapData[x][y].closeIndex = -1
	mapManager.printPathList[x][y] = "."
}

func (mapManager *MapManager) smoothPath() {
	if len(mapManager.readySmoothPathList) <= 2 {
		fmt.Println("just 2 node,not need smooth path")
		mapManager.afterUseLessPathList = append(mapManager.afterUseLessPathList, mapManager.readySmoothPathList...)
		return
	}
	//-----------------------------------------------------delete useless node(node >= 3)-----------------------------------------------------
	//add Start node into afterUseLessPathList
	mapManager.afterUseLessPathList = append(mapManager.afterUseLessPathList, mapManager.readySmoothPathList[0])
	//set an unreal start dir,same to next
	mapManager.dirPathList = append(mapManager.dirPathList, InitDirNode(mapManager.readySmoothPathList[1].X-mapManager.readySmoothPathList[0].X, mapManager.readySmoothPathList[1].Y-mapManager.readySmoothPathList[0].Y))
	//set nowDir same as dirPathList[0]
	mapManager.nowDirX = mapManager.dirPathList[0].dirX
	mapManager.nowDirY = mapManager.dirPathList[0].dirY
	//set all dir
	for i := 1; i < len(mapManager.readySmoothPathList); i++ {
		mapManager.dirPathList = append(mapManager.dirPathList,
			InitDirNode(mapManager.readySmoothPathList[i].X-mapManager.readySmoothPathList[i-1].X, mapManager.readySmoothPathList[i].Y-mapManager.readySmoothPathList[i-1].Y))
		//save ImportantInflection when delete useless node
		if mapManager.checkImportantInflectionIndex(mapManager.readySmoothPathList[i].X, mapManager.readySmoothPathList[i].Y) {
			mapManager.temporaryImportantIndexOfUselessNode = append(mapManager.temporaryImportantIndexOfUselessNode, mapManager.readySmoothPathList[i])
		}
	}
	//S
	// *E
	//S already add into afterUseLessPathList,if not same dir,add node[index-1] into afterUseLessPathList
	for index, val := range mapManager.dirPathList {
		//if not ! same dir,change nowDir
		if !(mapManager.nowDirX == val.dirX && mapManager.nowDirY == val.dirY) {
			mapManager.nowDirX = mapManager.dirPathList[index].dirX
			mapManager.nowDirY = mapManager.dirPathList[index].dirY
			//cut temporaryDeleteIndexPathList,only save start index and end index
			mapManager.afterUseLessPathList = append(mapManager.afterUseLessPathList, mapManager.readySmoothPathList[index-1])
		}
	}
	//finally add End node into afterUseLessPathList
	mapManager.afterUseLessPathList = append(mapManager.afterUseLessPathList, mapManager.readySmoothPathList[len(mapManager.readySmoothPathList)-1])
	var J = 0
	for i := 0; i < len(mapManager.afterUseLessPathList); i++ {
		//add next
		mapManager.FinalPathList = append(mapManager.FinalPathList, mapManager.afterUseLessPathList[i])
		//
		if i != len(mapManager.readySmoothPathList)-1 {
			for j := J; j < len(mapManager.temporaryImportantIndexOfUselessNode); j++ {
				//if not same to i and i+1
				if mapManager.afterUseLessPathList[i] != mapManager.temporaryImportantIndexOfUselessNode[j] &&
					mapManager.afterUseLessPathList[i+1] != mapManager.temporaryImportantIndexOfUselessNode[j] {
					if mapManager.isPointOnLineBetweenOptimized(
						mapManager.afterUseLessPathList[i].X, mapManager.afterUseLessPathList[i].Y,
						mapManager.afterUseLessPathList[i+1].X, mapManager.afterUseLessPathList[i+1].Y,
						mapManager.temporaryImportantIndexOfUselessNode[j].X, mapManager.temporaryImportantIndexOfUselessNode[j].Y) {
						//add temporaryImportantIndexOfUselessNode,not add same node
						mapManager.FinalPathList = append(mapManager.FinalPathList, mapManager.temporaryImportantIndexOfUselessNode[j])
						//if temporaryImportantIndexOfUselessNode is last one,stop add temporaryImportantIndexOfUselessNode
						if j == len(mapManager.temporaryImportantIndexOfUselessNode)-1 {
							J = len(mapManager.temporaryImportantIndexOfUselessNode)
						}
					} else {
						//if this time,first temporaryImportantIndexOfUselessNode is not PointOnLineBetweenOptimized
						//next time,begin from same j
						J = j
						break
					}
				} else {
					//same node
					//next J
					J = j + 1
					break
				}
			}
		}
	}
	//-----------------------------------------------------set importantFinalInflection AllFinalInflection Index-----------------------------------------------------
	//fmt.Println("check afterUseLessPathList:")
	//set ImportantInflectionIndex and UnimportantInflectionIndex about afterUseLessPathList
	//(is not including 0 index and end index)
	for i := 1; i < len(mapManager.FinalPathList)-1; i++ {
		//fmt.Println(
		//	i, " -> ", " x: ", mapManager.afterUseLessPathList[i].x,
		//	" y: ", mapManager.afterUseLessPathList[i].y,
		//	" nodeType: ", mapManager.afterUseLessPathList[i].nodeType)
		//set important inflectionIndex
		if mapManager.checkImportantInflectionIndex(mapManager.FinalPathList[i].X, mapManager.FinalPathList[i].Y) {
			//is important inflectionIndex
			mapManager.importantFinalInflectionIndex = append(mapManager.importantFinalInflectionIndex, i)
		}
		//set AllFinal InflectionIndex
		mapManager.allFinalInflectionIndex = append(mapManager.allFinalInflectionIndex, i)
	}
	//like [1] = [[0],[1]] || [2] = [[0,1]]
	for i := 1; i <= len(mapManager.importantFinalInflectionIndex); i++ {
		mapManager.SmoothValType.importantCombinationsFinalIndexMap[i] = mapManager.generateCombinations(len(mapManager.importantFinalInflectionIndex)-1, i, true)
	}
	for i := 1; i <= len(mapManager.allFinalInflectionIndex); i++ {
		mapManager.SmoothValType.allCombinationsFinalIndexMap[i] = mapManager.generateCombinations(len(mapManager.allFinalInflectionIndex)-1, i, false)
	}
	//-----------------------------------------------------recursive Obstacle Check-----------------------------------------------------
	//smoothBestWay
	mapManager.smoothBestWay()
	//
	//fmt.Println("check SmoothFinalIndex:")
	//for _, val := range mapManager.SmoothValType.SmoothFinalIndex {
	//	fmt.Println(" index ", val)
	//}
	//for _, val := range mapManager.SmoothValType.obstacleJudgeList {
	//	mapManager.printPathList[val.x][val.y] = "o"
	//}
	//fmt.Println("check obstacleJudge Map:")
	//for i := 0; i < mapManager.rows; i++ {
	//	fmt.Println()
	//	for j := 0; j < mapManager.cols; j++ {
	//		fmt.Print(mapManager.printPathList[i][j])
	//	}
	//}
}

func (mapManager *MapManager) isPointOnLineBetweenOptimized(x1, y1, x2, y2, x3, y3 int) bool {
	// 判断是否共线
	if (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1) != 0 {
		return false
	}

	// 判断是否位于两点之间
	if (x3 >= x1 && x3 <= x2) || (x3 >= x2 && x3 <= x1) {
		if (y3 >= y1 && y3 <= y2) || (y3 >= y2 && y3 <= y1) {
			return true
		}
	}
	return false
}
func (mapManager *MapManager) smoothBestWay() {
	//success H temporary h
	h, H := 0, 0
	successCombinationsPath := false
	//firstly:check importantCombinationsFinalIndexMap:[2,7] , map: => 1:[[2][7]] 2:[[2,7]]
	if len(mapManager.SmoothValType.importantCombinationsFinalIndexMap) > 0 {
		for i := 1; i <= len(mapManager.SmoothValType.importantCombinationsFinalIndexMap); i++ {
			//[[2][7]] => val: [2] , [7]
			for _, val := range mapManager.SmoothValType.importantCombinationsFinalIndexMap[i] {
				//?? 错的 smoothFinalIndex会被重置
				if successCombinationsPath, h = mapManager.checkCombinationsPath(val, 0, len(mapManager.FinalPathList)-1); successCombinationsPath {
					//get smaller h than H
					if H != 0 {
						if h < H {
							H = h
							mapManager.SmoothValType.SmoothFinalIndex = mapManager.SmoothValType.SmoothFinalIndex[:0]
							mapManager.SmoothValType.SmoothFinalIndex = append(mapManager.SmoothValType.SmoothFinalIndex, 0)
							mapManager.SmoothValType.SmoothFinalIndex = append(mapManager.SmoothValType.SmoothFinalIndex, val...)
							mapManager.SmoothValType.SmoothFinalIndex = append(mapManager.SmoothValType.SmoothFinalIndex, len(mapManager.FinalPathList)-1)
						}
					} else { //first success , save into H
						H = h
						mapManager.SmoothValType.SmoothFinalIndex = append(mapManager.SmoothValType.SmoothFinalIndex, 0)
						mapManager.SmoothValType.SmoothFinalIndex = append(mapManager.SmoothValType.SmoothFinalIndex, val...)
						mapManager.SmoothValType.SmoothFinalIndex = append(mapManager.SmoothValType.SmoothFinalIndex, len(mapManager.FinalPathList)-1)
					}
				}
			}
			//success find smaller path,quit loop,you can check SmoothFinalIndex
			if H != 0 {
				return
			}
		}
	}
	//secondly:check allCombinationsFinalIndexMap(finalPath Combinations):
	if len(mapManager.SmoothValType.allCombinationsFinalIndexMap) > 0 {
		for i := 1; i <= len(mapManager.SmoothValType.allCombinationsFinalIndexMap); i++ {
			//[1,2,3,4,5,6,7,8,9,10] => val: [2] , [7]
			for _, val := range mapManager.SmoothValType.allCombinationsFinalIndexMap[i] {
				//?? 错的 smoothFinalIndex会被重置
				if successCombinationsPath, h = mapManager.checkCombinationsPath(val, 0, len(mapManager.FinalPathList)-1); successCombinationsPath {
					//get smaller h than H
					if H != 0 {
						if h < H {
							H = h
							mapManager.SmoothValType.SmoothFinalIndex = mapManager.SmoothValType.SmoothFinalIndex[:0]
							mapManager.SmoothValType.SmoothFinalIndex = append(mapManager.SmoothValType.SmoothFinalIndex, 0)
							mapManager.SmoothValType.SmoothFinalIndex = append(mapManager.SmoothValType.SmoothFinalIndex, val...)
							mapManager.SmoothValType.SmoothFinalIndex = append(mapManager.SmoothValType.SmoothFinalIndex, len(mapManager.FinalPathList)-1)
						}
					} else { //first success , save into H
						H = h
						mapManager.SmoothValType.SmoothFinalIndex = append(mapManager.SmoothValType.SmoothFinalIndex, 0)
						mapManager.SmoothValType.SmoothFinalIndex = append(mapManager.SmoothValType.SmoothFinalIndex, val...)
						mapManager.SmoothValType.SmoothFinalIndex = append(mapManager.SmoothValType.SmoothFinalIndex, len(mapManager.FinalPathList)-1)
					}
				}
			}
			//success find smaller path,quit loop,you can check SmoothFinalIndex
			if H != 0 {
				return
			}
		}
	}
}

// h is success path cost
func (mapManager *MapManager) checkCombinationsPath(combinations []int, startIndex, endIndex int) (bool, int) {
	h := 0
	//0 to combinations
	current := startIndex
	for _, combinationsIndexOfFinalPath := range combinations {
		//check map has HVal first
		if ok, H := mapManager.getAllObstacleFinalIndexHValMap(current, combinationsIndexOfFinalPath); ok {
			//set h+= and SmoothFinalIndex
			h += H
			current = combinationsIndexOfFinalPath
			continue
		} else {
			//if not,obstacleJudge calculate H val
			if mapManager.obstacleJudge(
				mapManager.mapData[mapManager.FinalPathList[current].X][mapManager.FinalPathList[current].Y],
				mapManager.mapData[mapManager.FinalPathList[combinationsIndexOfFinalPath].X][mapManager.FinalPathList[combinationsIndexOfFinalPath].Y]) {
				//calculate H
				H = mapManager.getNeighborCost(
					mapManager.FinalPathList[current].X, mapManager.FinalPathList[current].Y,
					mapManager.FinalPathList[combinationsIndexOfFinalPath].X, mapManager.FinalPathList[combinationsIndexOfFinalPath].Y)
				//set map  map[int]map[int]int 1 to 1 to 1
				mapManager.SmoothValType.successAllObstacleFinalIndexHValMap[current] = make(map[int]int, 1)
				mapManager.SmoothValType.successAllObstacleFinalIndexHValMap[current][combinationsIndexOfFinalPath] = H //!!! H
				//set h+= and SmoothFinalIndex
				h += H
				//current change to now combinationsIndexOfFinalPath
				current = combinationsIndexOfFinalPath
			} else {
				//if obstacleJudge filed , means no way
				return false, 0
			}
		}
	}
	//last combination to endIndex
	if ok, H := mapManager.getAllObstacleFinalIndexHValMap(current, endIndex); ok {
		h += H
		return true, h
	} else {
		if mapManager.obstacleJudge(
			mapManager.mapData[mapManager.FinalPathList[current].X][mapManager.FinalPathList[current].Y],
			mapManager.mapData[mapManager.FinalPathList[endIndex].X][mapManager.FinalPathList[endIndex].Y]) {
			//
			H = mapManager.getNeighborCost(
				mapManager.FinalPathList[current].X, mapManager.FinalPathList[current].Y,
				mapManager.FinalPathList[endIndex].X, mapManager.FinalPathList[endIndex].Y)
			//set map  map[int]map[int]int 1 to 1 to 1
			mapManager.SmoothValType.successAllObstacleFinalIndexHValMap[current] = make(map[int]int, 1)
			mapManager.SmoothValType.successAllObstacleFinalIndexHValMap[current][endIndex] = H //!!! H
			//
			h += H
			return true, h
		} else {
			//if obstacleJudge filed , means no way
			mapManager.SmoothValType.SmoothFinalIndex = mapManager.SmoothValType.SmoothFinalIndex[:0]
			return false, 0
		}
	}
}

// return ok
func (mapManager *MapManager) getAllObstacleFinalIndexHValMap(startIndex, endIndex int) (bool, int) {
	if innerMap, ok := mapManager.SmoothValType.successAllObstacleFinalIndexHValMap[startIndex]; ok {
		if value, ok := innerMap[endIndex]; ok {
			return true, value
		}
	}
	return false, 0 // 如果未找到，返回默认值和 false
}

func (mapManager *MapManager) getNeighborCost(x1, y1, x2, y2 int) int {
	dx := x1 - x2
	dy := y1 - y2
	if dx < 0 {
		dx = -dx
	}
	if dy < 0 {
		dy = -dy
	}
	return dx + dy
}

func (mapManager *MapManager) getSmaller(a, b int) int {
	if a < b {
		return a
	} else {
		return b
	}
}
func (mapManager *MapManager) getLarger(a, b int) int {
	if a > b {
		return a
	} else {
		return b
	}
}

// Generate Combinations --n:len k:want Combinations
func (mapManager *MapManager) generateCombinations(n, k int, ip bool) [][]int {
	var result [][]int
	// 使用一个栈来保存当前组合的状态
	var stack []int
	// 初始状态：从 0 开始
	start := 0
	//
	for {
		// 如果当前组合的长度等于 k，说明找到一个有效的组合
		if len(stack) == k {
			// 将当前组合复制到结果中
			combination := make([]int, k)
			//ip is true,importantFinalInflectionIndex else afterUseLessPathList
			if ip {
				for index, val := range stack {
					combination[index] = mapManager.importantFinalInflectionIndex[val]
				}
			} else {
				for index, val := range stack {
					combination[index] = mapManager.allFinalInflectionIndex[val]
				}
			}
			//
			result = append(result, combination)
			// 回溯：弹出栈顶元素，尝试下一个可能的数字
			start = stack[len(stack)-1] + 1
			stack = stack[:len(stack)-1]
		}

		// 如果 start 超过 n，说明当前分支已经遍历完，需要回溯
		if start > n {
			// 如果栈为空，说明所有组合已经生成完毕
			if len(stack) == 0 {
				break
			}
			// 回溯：弹出栈顶(end)元素，尝试下一个可能的数字
			start = stack[len(stack)-1] + 1
			stack = stack[:len(stack)-1]
		} else {
			// importantFinalInflectionIndex对应索引值添加
			stack = append(stack, start)
			start++
		}
	}
	//
	return result
}

func (mapManager *MapManager) checkImportantInflectionIndex(x int, y int) bool {
	var times = 0
	//------------------------check k=1/-1-------------
	if x+1 >= 0 && y+1 >= 0 && x+1 < mapManager.rows && y+1 < mapManager.cols {
		//is important inflectionIndex
		if mapManager.mapData[x+1][y+1].nodeType == 1 {
			times++
		}
	}
	if x+1 >= 0 && y-1 >= 0 && x+1 < mapManager.rows && y-1 < mapManager.cols {
		//is important inflectionIndex
		if mapManager.mapData[x+1][y-1].nodeType == 1 {
			times++
		}
	}
	if x-1 >= 0 && y-1 >= 0 && x-1 < mapManager.rows && y-1 < mapManager.cols {
		//is important inflectionIndex
		if mapManager.mapData[x-1][y-1].nodeType == 1 {
			times++
		}
	}
	if x-1 >= 0 && y+1 >= 0 && x-1 < mapManager.rows && y+1 < mapManager.cols {
		//is important inflectionIndex
		if mapManager.mapData[x-1][y+1].nodeType == 1 {
			times++
		}
	}
	//------------------------check k=0---------------
	if x+1 >= 0 && y >= 0 && x+1 < mapManager.rows && y < mapManager.cols {
		//is important inflectionIndex
		if mapManager.mapData[x+1][y].nodeType == 1 {
			times++
		}
	}
	if x >= 0 && y+1 >= 0 && x < mapManager.rows && y+1 < mapManager.cols {
		//is important inflectionIndex
		if mapManager.mapData[x][y+1].nodeType == 1 {
			times++
		}
	}
	if x-1 >= 0 && y >= 0 && x-1 < mapManager.rows && y < mapManager.cols {
		//is important inflectionIndex
		if mapManager.mapData[x-1][y].nodeType == 1 {
			times++
		}
	}
	if x >= 0 && y-1 >= 0 && x < mapManager.rows && y-1 < mapManager.cols {
		//is important inflectionIndex
		if mapManager.mapData[x][y-1].nodeType == 1 {
			times++
		}
	}
	//ImportantInflection just one Obstacle around k=1/-1
	if times == 1 {
		return true
	} else {
		return false
	}
}

func (mapManager *MapManager) obstacleJudge(startNode *Node, endNode *Node) bool {
	//mapManager.SmoothValType.temporaryJudgeList = mapManager.SmoothValType.temporaryJudgeList[:0]
	//if y is same or x is same
	if startNode.X == endNode.X {
		if (startNode.Y-endNode.Y) == 1 || (endNode.Y-startNode.Y) == 1 {
			return true
		}
		//min y is left
		//for i := int(math.Min(float64(startNode.y), float64(endNode.y))) + 1; i < int(math.Max(float64(startNode.y), float64(endNode.y))); i++ {
		for i := mapManager.getSmaller(startNode.Y, endNode.Y) + 1; i < mapManager.getLarger(startNode.Y, endNode.Y); i++ {
			//if wall
			if mapManager.mapData[startNode.X][i].nodeType == 1 {
				return false
			}
			//mapManager.SmoothValType.obstacleJudgeList = append(mapManager.SmoothValType.obstacleJudgeList, mapManager.mapData[startNode.x][i])
		}
		return true
	}
	if startNode.Y == endNode.Y {
		if (startNode.X-endNode.X) == 1 || (endNode.X-startNode.X) == 1 {
			return true
		}
		//min x is up
		//for i := int(math.Min(float64(startNode.x), float64(endNode.x))) + 1; i < int(math.Max(float64(startNode.x), float64(endNode.x))); i++ {
		for i := mapManager.getSmaller(startNode.X, endNode.X) + 1; i < mapManager.getLarger(startNode.X, endNode.X); i++ {
			//if wall
			if mapManager.mapData[i][startNode.Y].nodeType == 1 {
				return false
			}
		}
		return true
	}
	//if (1,1) or (1,-1)
	//S(0,0) (0,1) 	//(0,0) S(0,1)
	//(1,0) E(1,1) 	//E(1,0) (1,1)
	if (endNode.X-startNode.X == 1 && endNode.Y-startNode.Y == 1) || (endNode.X-startNode.X == 1 && endNode.Y-startNode.Y == -1) {

		if mapManager.mapData[startNode.X][endNode.Y].nodeType == 1 ||
			mapManager.mapData[endNode.X][startNode.Y].nodeType == 1 {
			return false
		}
		return true
	}
	//if (-1,1) or (-1,-1)
	//E(0,0) (0,1) 	//(0,0) E(0,1)
	//(1,0) S(1,1)  //S(1,0) (1,1)
	if (endNode.X-startNode.X == -1 && endNode.Y-startNode.Y == -1) || (endNode.X-startNode.X == -1 && endNode.Y-startNode.Y == 1) {
		//
		if mapManager.mapData[endNode.X][startNode.Y].nodeType == 1 ||
			mapManager.mapData[startNode.X][endNode.Y].nodeType == 1 {
			return false
		}
		return true
	}
	//use math y=kx+b obstacleJudge System
	if mapManager.newObstacleJudge(startNode, endNode) {
		return true
	} else {
		return false
	}

}

// DeleteSlice3 删除指定元素。
//func DeleteSlice3(s []int, elem int) []int {
//	j := 0
//	for _, v := range s {
//		if v != elem {
//			s[j] = v
//			j++
//		}
//	}
//	return s[:j]
//}

//10. 使用 copy 快速删除
//Go 语言内置的 copy 函数也可以用来快速删除切片元素:
//func remove(slice []int, i int) []int {
//	copy(slice[i:], slice[i+1:])
//	return slice[:len(slice)-1]
//}
//测试一下:
//data := []int{0, 1, 2, 3}
//remove(data, 2) // [0, 1, 3]

func (mapManager *MapManager) pathFind(x1, y1, x2, y2 int) {
	//heap.Init(&mapManager.openList)
	//start path find:8 direction
	offsetX := 0
	offsetY := 0
	f, g, h := 0.0, 0.0, 0.0
	mapManager.openListChangeFlag = false
	for index, offset := range rangeOffset {
		//may eight times
		//{0, 1}right 		{0, -1}left 	 	 {-1, 0}up 	     	 {1, 0}down
		//{-1, 1}right up 	{1, 1}right down 	 {-1, -1}left up 	 {1, -1}left down
		f, g, h = 0.0, 0.0, 0.0
		offsetX = x1 + offset[0]
		offsetY = y1 + offset[1]
		//judge boundary,map boundary
		if offsetX >= 0 && offsetY >= 0 && offsetX < mapManager.rows && offsetY < mapManager.cols {
			//already in openList or closedList or is wall
			if mapManager.mapData[offsetX][offsetY].nodeType == 1 || mapManager.mapData[offsetX][offsetY].open || mapManager.mapData[offsetX][offsetY].closed {
				continue
			}
			//not start node
			if offsetX == mapManager.startNodeX && offsetY == mapManager.startNodeY {
				continue
			}
			//if k=-1/1,judge 2 obstacle around of 4 status,if have 1 obstacle,continue
			if index >= 4 {
				//judge 4 status,if someone is obstacle,continue
				switch index {
				case 4:
					//! OFFSET
					//F !
					if offsetX >= 0 && offsetY-1 >= 0 && offsetX < mapManager.rows && offsetY-1 < mapManager.cols {
						if mapManager.mapData[offsetX][offsetY-1].nodeType == 1 {
							continue
						}
					}
					if offsetX+1 >= 0 && offsetY >= 0 && offsetX+1 < mapManager.rows && offsetY < mapManager.cols {
						if mapManager.mapData[offsetX+1][offsetY].nodeType == 1 {
							continue
						}
					}
				case 5:
					//F !
					//! OFFSET
					if offsetX-1 >= 0 && offsetY >= 0 && offsetX-1 < mapManager.rows && offsetY < mapManager.cols {
						if mapManager.mapData[offsetX-1][offsetY].nodeType == 1 {
							continue
						}
					}
					if offsetX >= 0 && offsetY-1 >= 0 && offsetX < mapManager.rows && offsetY-1 < mapManager.cols {
						if mapManager.mapData[offsetX][offsetY-1].nodeType == 1 {
							continue
						}
					}
				case 6:
					//OFFSET !
					//!      F
					if offsetX >= 0 && offsetY+1 >= 0 && offsetX < mapManager.rows && offsetY+1 < mapManager.cols {
						if mapManager.mapData[offsetX][offsetY+1].nodeType == 1 {
							continue
						}
					}
					if offsetX+1 >= 0 && offsetY >= 0 && offsetX+1 < mapManager.rows && offsetY < mapManager.cols {
						if mapManager.mapData[offsetX+1][offsetY].nodeType == 1 {
							continue
						}
					}
				case 7:
					//!      F
					//OFFSET !
					if offsetX-1 >= 0 && offsetY >= 0 && offsetX-1 < mapManager.rows && offsetY < mapManager.cols {
						if mapManager.mapData[offsetX-1][offsetY].nodeType == 1 {
							continue
						}
					}
					if offsetX >= 0 && offsetY+1 >= 0 && offsetX < mapManager.rows && offsetY+1 < mapManager.cols {
						if mapManager.mapData[offsetX][offsetY+1].nodeType == 1 {
							continue
						}
					}
				}
				//
				g += 1.4
			} else {
				g += 1
			}
			//if not in openList or closedList,judge wall
			if mapManager.mapData[offsetX][offsetY].nodeType == 0 {
				//g from this center node(offset father)
				g += mapManager.mapData[x1][y1].g
				//h from this offset node to end
				h = math.Abs(float64(x2-offsetX)) + math.Abs(float64(y2-offsetY))
				//calculate f = g + h
				f = g + h
				//save f,g,h
				mapManager.mapData[offsetX][offsetY].f = f
				mapManager.mapData[offsetX][offsetY].g = g
				mapManager.mapData[offsetX][offsetY].h = h
				//save father node
				mapManager.mapData[offsetX][offsetY].father = mapManager.mapData[x1][y1]
				//add into openList
				heap.Push(&mapManager.openList, mapManager.mapData[offsetX][offsetY])
				//mapManager.openList = append(mapManager.openList, mapManager.mapData[offsetX][offsetY])
				mapManager.mapData[offsetX][offsetY].open = true
				mapManager.openListChangeFlag = true
				//if is end node
				if offsetX == x2 && offsetY == y2 {
					//add End node into pathList
					//mapManager.pathList = append(mapManager.pathList, mapManager.mapData[x2][y2])
					mapManager.readySmoothPathList = append(mapManager.readySmoothPathList, mapManager.mapData[x2][y2])
					//add into closeList
					mapManager.closeList = append(mapManager.closeList, mapManager.mapData[x2][y2])
					//find path from closeList End node father
					mapManager.closePathIndex = len(mapManager.closeList) - 1
					var fatherNode = mapManager.closeList[mapManager.closePathIndex].father
					for {
						if fatherNode != nil {
							//mapManager.pathList = append(mapManager.pathList, fatherNode)
							mapManager.readySmoothPathList = append(mapManager.readySmoothPathList, fatherNode)
							fatherNode = fatherNode.father
						} else {
							//fmt.Println("success find path!")
							//var time1 = time.Now().UnixMicro()
							//reverseSlice(mapManager.pathList)
							reverseSlice(mapManager.readySmoothPathList)
							//mapManager.pathList assign to readySmoothPathList
							//copy(mapManager.readySmoothPathList, mapManager.pathList)
							//var time2 = time.Now().UnixMicro()
							//fmt.Println("reverseSlice time 微秒: ", time2-time1)
							//
							for _, val := range mapManager.readySmoothPathList {
								//fmt.Println(index, " -> ", " x: ", val.x, " y: ", val.y, " nodeType: ", val.nodeType)
								mapManager.printPathList[val.X][val.Y] = "*"
							}
							mapManager.printPathList[x2][y2] = "E"
							mapManager.printPathList[mapManager.startNodeX][mapManager.startNodeY] = "S"
							//set pathFindFlag true,success find path
							mapManager.pathFindFlag = true
							return
						}
					}
				}
			}
		}
	}
	///len >0 && openListChangeFlag
	//if mapManager.openListChangeFlag {
	//	SortTime1 = time.Now().UnixMicro()
	//	//slices.so
	//	//sort openList
	//	sort.Slice(mapManager.openList, func(i, j int) bool {
	//		return mapManager.openList[i].f > mapManager.openList[j].f
	//	})
	//	//
	//	SortTime2 = time.Now().UnixMicro()
	//	SortTime += SortTime2 - SortTime1
	//}
	if len(mapManager.openList) > 0 {
		//add latest node into closeList from openList,remove latest node from openList,set flag
		//mapManager.openList[len(mapManager.openList)-1].open = false
		//mapManager.openList[len(mapManager.openList)-1].closed = true
		//mapManager.closeList = append(mapManager.closeList, mapManager.openList[len(mapManager.openList)-1])
		//mapManager.openList = mapManager.openList[:len(mapManager.openList)-1]
		//
		var node = heap.Pop(&mapManager.openList).(*Node)
		node.open = false
		node.closed = true
		mapManager.closeList = append(mapManager.closeList, node)
		//recursion pathFind method,start is closeList latest node
		mapManager.pathFind(mapManager.closeList[len(mapManager.closeList)-1].X, mapManager.closeList[len(mapManager.closeList)-1].Y, x2, y2)
	} else {
		//if not node in openList now
		fmt.Println("openList is empty, no way find!")
	}
	return
}

//func (mapManager *MapManager) sortFunc(i, j int) bool {
//	return mapManager.openList[i].f > mapManager.openList[j].f
//}

// 任意位置插入数字类型的元素 注意这个！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
// @param slice []int 将指定元素插入的切片
// @param num int 指定元素
// @param index int 插入的指定位置
//func arrayInsertElement(slice []int, num int, index int) []int {
//	slice = append(slice[:index], append([]int{num}, slice[index:]...)...)
//	return slice
//}

func (mapManager *MapManager) newObstacleJudge(startNode *Node, endNode *Node) bool {
	mapManager.SmoothValType.k = float64(endNode.Y-startNode.Y) / float64(endNode.X-startNode.X)
	mapManager.SmoothValType.b = (float64(startNode.Y) + 0.5) - (mapManager.SmoothValType.k * (float64(startNode.X) + 0.5))
	//check left and right
	for x := mapManager.getSmaller(startNode.X, endNode.X) + 1; x <= mapManager.getLarger(startNode.X, endNode.X); x++ {
		realY := (mapManager.SmoothValType.k * float64(x)) + mapManager.SmoothValType.b
		if !mapManager.checkYKXBForY(x, realY) {
			return false
		}
	}
	//check up and down
	for y := mapManager.getSmaller(startNode.Y, endNode.Y) + 1; y <= mapManager.getLarger(startNode.Y, endNode.Y); y++ {
		realX := (float64(y) - mapManager.SmoothValType.b) / mapManager.SmoothValType.k
		if !mapManager.checkYKXBForX(y, realX) {
			return false
		}
	}
	return true
}

func (mapManager *MapManager) checkNodeIsObstacle(x, y int) bool {
	return mapManager.mapData[x][y].nodeType == 1
}

func (mapManager *MapManager) checkYKXBForY(x int, realY float64) bool {
	//left right
	floor := math.Floor(realY)
	ceil := math.Ceil(realY)
	if ceil-realY <= 0.01 {
		//close is ceil,check 4 obstacle
		if mapManager.checkNodeIsObstacle(x, int(ceil)) ||
			mapManager.checkNodeIsObstacle(x-1, int(ceil)) ||
			mapManager.checkNodeIsObstacle(x, int(ceil)-1) ||
			mapManager.checkNodeIsObstacle(x-1, int(ceil)-1) {
			return false
		}
	} else if realY-floor <= 0.01 {
		//close is floor,check 4 obstacle
		if mapManager.checkNodeIsObstacle(x, int(floor)) ||
			mapManager.checkNodeIsObstacle(x-1, int(floor)) ||
			mapManager.checkNodeIsObstacle(x, int(floor)-1) ||
			mapManager.checkNodeIsObstacle(x-1, int(floor)-1) {
			return false
		}
	} else {
		//close is center,check 2 obstacle(left right)
		if mapManager.checkNodeIsObstacle(x, int(realY)) ||
			mapManager.checkNodeIsObstacle(x-1, int(realY)) {
			return false
		}
	}
	return true
}

func (mapManager *MapManager) checkYKXBForX(y int, realX float64) bool {
	//left right
	floor := math.Floor(realX)
	ceil := math.Ceil(realX)
	if ceil-realX <= 0.01 {
		//close is ceil,check 4 obstacle
		if mapManager.checkNodeIsObstacle(int(ceil), y) ||
			mapManager.checkNodeIsObstacle(int(ceil)-1, y) ||
			mapManager.checkNodeIsObstacle(int(ceil), y-1) ||
			mapManager.checkNodeIsObstacle(int(ceil)-1, y-1) {
			return false
		}
	} else if realX-floor <= 0.01 {
		//close is floor,check 4 obstacle
		if mapManager.checkNodeIsObstacle(int(floor), y) ||
			mapManager.checkNodeIsObstacle(int(floor)-1, y) ||
			mapManager.checkNodeIsObstacle(int(floor), y-1) ||
			mapManager.checkNodeIsObstacle(int(floor)-1, y-1) {
			return false
		}
	} else {
		//close is center,check 2 obstacle(up down)
		if mapManager.checkNodeIsObstacle(int(realX), y) ||
			mapManager.checkNodeIsObstacle(int(realX), y-1) {
			return false
		}
	}
	return true
}

type printTimeToken struct {
	ResetTimeCost int64
	PathFindCost  int64
	EndTime       int64
	StartTime     int64
}
