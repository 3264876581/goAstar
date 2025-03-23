package DataMgr

import (
	"sort"
	"strconv"
	"fmt"
	"math"
	"time"
)

type Node struct {
	X, Y        int16   // 小于 2000，可以用 uint16（2 字节）
	nodeType    uint8   // 0 或 1，可以用 uint8（1 字节）
	nodeRoadNum int8    // 小于 100，可以用 int8（1 字节）
	f, g, h     float32 // 降为 float32（4 字节）
	father      *Node   // 指针（8 字节）
	open        bool    // bool（1 字节）
	closed      bool    // bool（1 字节）
}

type DirNode struct {
	dirX, dirY int16
}

func InitNode(x, y int16, nodeType uint8) *Node {
	return &Node{
		X:           x,
		Y:           y,
		nodeType:    0, //0:road,1:wall
		nodeRoadNum: 2, //尝试2
		f:           0,
		g:           0,
		h:           0,
		father:      nil,
		open:        false,
		closed:      false,
	}
}

func InitDirNode(dirx, diry int16) *DirNode {
	return &DirNode{
		dirX: dirx,
		dirY: diry,
	}
}

//// PriorityQueue 实现一个最小堆
//type PriorityQueue []*Node
//
//func (pq PriorityQueue) Len() int { return len(pq) }
//func (pq PriorityQueue) Less(i, j int) bool {
//	// 首先按f排序，f相同则按h排序,h小的在前面
//	//if pq[i].f == pq[j].f {
//	//	return pq[i].h <= pq[j].h
//	//}
//	//只按f排序
//	return pq[i].f < pq[j].f
//}
//func (pq PriorityQueue) Swap(i, j int)       { pq[i], pq[j] = pq[j], pq[i] }
//func (pq *PriorityQueue) Push(x interface{}) { *pq = append(*pq, x.(*Node)) }
//func (pq *PriorityQueue) Pop() interface{} {
//	old := *pq
//	n := len(old)
//	item := old[n-1]
//	*pq = old[:n-1]
//	return item
//}

type MapManager struct {
	mapData    [][]*Node
	rows       int16
	cols       int16
	startNodeX int16
	startNodeY int16
	//openList   PriorityQueue
	openList  []*Node
	closeList []*Node
	judgeList []*Node
	//heap       *Heap
	//
	closePathIndex                       int
	printPathList                        [][]string
	pathFindFlag                         bool
	readySmoothPathList                  []*Node
	dirPathList                          []*DirNode
	nowDirX                              int16
	nowDirY                              int16
	temporaryImportantIndexOfUselessNode []*Node
	afterUseLessPathList                 []*Node
	repeatAfterUseLessPathList           []*Node
	FinalPathList                        []*Node
	importantFinalInflectionIndex        []int
	allFinalInflectionIndex              []int
	SmoothValType                        *smoothVal
	PrintTime                            *printTimeToken
	recordSaveIndexSlice                 map[int]interface{}
	sortSlice                            []int
	temRecordSaveIndexSlice              map[int]interface{}
	temSortSlice                         []int
	temRecordNodeSlice                   []*Node
	deleteTime                           int
}

type smoothVal struct {
	SmoothFinalIndex []int
	startIndex       int
	midIndex         int
	endIndex         int
	//obstacleJudgeList                   []*Node
	importantCombinationsFinalIndexMap  map[int][][]int         //key:[Combinations],value:[[][][]]
	allCombinationsFinalIndexMap        map[int][][]int         //key:[Combinations],value:[[][][]]
	successAllObstacleFinalIndexHValMap map[int]map[int]float64 //key:[startFinalIndex],value:[endFinalIndex],son value:H(Manhattan cost)
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

func reverseSlice(arr *[]*Node) {
	lenArr := len(*arr)
	for i := 0; i < lenArr/2; i++ {
		(*arr)[i], (*arr)[lenArr-1-i] = (*arr)[lenArr-1-i], (*arr)[i]
	}
	//for i, j := 0, len(*arr)-1; i < j; i, j = i+1, j-1 {
	//	(*arr)[i], (*arr)[j] = (*arr)[j], (*arr)[i]
	//}
}

func (mapManager *MapManager) SetObstacle(xIndex, yIndex int) {
	mapManager.mapData[xIndex][yIndex].nodeType = 1 //Obstacle
	mapManager.printPathList[xIndex][yIndex] = "!"
}

func (mapManager *MapManager) SetFastRoad(xIndex, yIndex, roadNum int8) {
	if mapManager.mapData[xIndex][xIndex].nodeType == 0 {
		mapManager.mapData[xIndex][yIndex].nodeRoadNum = roadNum //Obstacle
	}
}

func (mapManager *MapManager) SetRoad(xIndex, yIndex int) {
	mapManager.mapData[xIndex][yIndex].nodeType = 0 //road
	mapManager.printPathList[xIndex][yIndex] = "."
}

// NewMapManager
// @width :rows
// @height :cols
// return :*MapManager
func NewMapManager(width, height int16) *MapManager {
	time1 := time.Now().UnixMilli()
	rows, cols := height, width // 定义行数和列数
	twoDSlice := make([][]*Node, rows)
	printList := make([][]string, rows)
	// 初始化二维切片的每一行
	for i := range twoDSlice {
		twoDSlice[i] = make([]*Node, cols)
		printList[i] = make([]string, cols)
	}
	var i int16 = 0
	var j int16 = 0
	for i = 0; i < rows; i++ {
		for j = 0; j < cols; j++ {
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
		//openList:  make(PriorityQueue, 0, 1024*128),
		openList:  make([]*Node, 0, 1024*128),
		closeList: make([]*Node, 0, 1024*4),
		judgeList: make([]*Node, 0, 1024*128),
		//heap: &Heap{},
		//path and smooth path
		closePathIndex:                       -1,
		printPathList:                        printList,
		pathFindFlag:                         false,
		readySmoothPathList:                  make([]*Node, 0, 1024*4),
		dirPathList:                          make([]*DirNode, 0, 1024*4),
		nowDirX:                              0,
		nowDirY:                              0,
		temporaryImportantIndexOfUselessNode: make([]*Node, 0, 32),
		afterUseLessPathList:                 make([]*Node, 0, 1024),
		repeatAfterUseLessPathList:           make([]*Node, 0, 128),
		FinalPathList:                        make([]*Node, 0, 64),
		importantFinalInflectionIndex:        make([]int, 0, 64),
		allFinalInflectionIndex:              make([]int, 0, 64),
		recordSaveIndexSlice:                 make(map[int]interface{}, 128),
		sortSlice:                            make([]int, 0, 128),
		temRecordSaveIndexSlice:              make(map[int]interface{}, 64),
		temSortSlice:                         make([]int, 0, 64),
		temRecordNodeSlice:                   make([]*Node, 0, 64),
		deleteTime:                           0,
		SmoothValType: &smoothVal{
			//obstacleJudgeList:                   make([]*Node, 0, 2),
			importantCombinationsFinalIndexMap:  make(map[int][][]int, 64),         //key:[Combinations],value:[[][][]]
			allCombinationsFinalIndexMap:        make(map[int][][]int, 64),         //key:[Combinations],value:[[][][]]
			successAllObstacleFinalIndexHValMap: make(map[int]map[int]float64, 64), //key:[startFinalIndex],value:[endFinalIndex],son value:H
			SmoothFinalIndex:                    make([]int, 0, 1024*4),
			temporaryJudgeList:                  make([]*Node, 0, 1024*4),
			allPass:                             false},
		PrintTime: &printTimeToken{},
	}
}

var rangeOffset = [][]int16{
	{0, 1},  // up
	{0, -1}, // down
	{-1, 0}, // left
	{1, 0},  // right
	//{0, 2},  // up
	//{0, -2}, // down
	//{-2, 0}, // left
	//{2, 0},  // right
	{-1, 1},  //left up
	{1, 1},   //right up
	{-1, -1}, //left down
	{1, -1},  //right down
	//*9*10*
	//8***11
	//**o**
	//15***12
	//*14*13*
	//{-1, -2}, //8
	//{-2, -1}, //9
	//{-2, 1},  //10
	//{-1, 2},  //11
	////
	//{1, 2},  //12
	//{2, 1},  //13
	//{2, -1}, //14
	//{1, -2}, //15
}

func (mapManager *MapManager) printMap() {
	for i := int16(0); i < mapManager.rows; i++ {
		fmt.Println()
		for j := int16(0); j < mapManager.cols; j++ {
			fmt.Print(mapManager.printPathList[i][j])
		}
	}
	fmt.Println()
}

// PathFind true:Success Find a road
// result: SmoothFinalIndex(some index of FinalPathList slice)
// node: FinalPathList(Inflection node)
func (mapManager *MapManager) PathFind(x1, y1, x2, y2 int, printResultFlag, printMapFlag, printTimeTokenFlag bool) bool {
	mapManager.PrintTime.AllPathFindCost = time.Now().UnixMicro()
	mapManager.PrintTime.ResetTimeCost = time.Now().UnixMicro()
	//boundary status
	if x1 < 0 || y1 < 0 || x2 < 0 || y2 < 0 || int16(x1) >= mapManager.rows || int16(y1) >= mapManager.cols || int16(x2) >= mapManager.rows || int16(y2) >= mapManager.cols {
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
		mapManager.PrintTime.AllPathFindCost = time.Now().UnixMicro() - mapManager.PrintTime.AllPathFindCost
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
			fmt.Printf("%-25s %d μs\n", "AllPathFindCost Token(no obstacle):", mapManager.PrintTime.AllPathFindCost)
		}
		//success obstacleJudge return true
		return true
	}
	//resetMapData
	mapManager.resetMapData(int16(x1), int16(y1))
	mapManager.PrintTime.ResetTimeCost = time.Now().UnixMicro() - mapManager.PrintTime.ResetTimeCost
	//pathFind
	mapManager.PrintTime.PathFindCost = time.Now().UnixMicro()
	mapManager.pathFind(int16(x1), int16(y1), int16(x2), int16(y2))
	mapManager.PrintTime.PathFindCost = time.Now().UnixMicro() - mapManager.PrintTime.PathFindCost
	//pathFind success
	if mapManager.pathFindFlag {
		//SmoothPath
		mapManager.smoothPath()
		mapManager.PrintTime.AllPathFindCost = time.Now().UnixMicro() - mapManager.PrintTime.AllPathFindCost
		//printMap
		if printMapFlag {
			for index, val := range mapManager.FinalPathList {
				mapManager.printPathList[val.X][val.Y] = strconv.Itoa(index)
			}
			for _, val := range mapManager.judgeList {
				mapManager.printPathList[val.X][val.Y] = "a"
			}
			mapManager.printMap()
		}
		//printResult
		if printResultFlag {
			mapManager.printResult()
		}
		//printTime
		if printTimeTokenFlag {
			fmt.Printf("%-25s %d μs\n", "ResetTimeCost Taken:", mapManager.PrintTime.ResetTimeCost)
			fmt.Printf("%-25s %d μs\n", "UseLessCost Taken:", mapManager.PrintTime.UseLessCost)
			fmt.Printf("%-25s %d μs\n", "SetCombinationNodeCost Taken:", mapManager.PrintTime.SetCombinationNodeCost)
			fmt.Printf("%-25s %d μs\n", "SmoothBestWay Taken:", mapManager.PrintTime.SmoothBestWay)
			fmt.Printf("%-25s %d μs\n", "PathFindCost Taken:", mapManager.PrintTime.PathFindCost)
			fmt.Printf("%-25s %d μs\n", "AllPathFindCost Taken:", mapManager.PrintTime.AllPathFindCost)
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
func (mapManager *MapManager) resetMapData(x1, y1 int16) {
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
	//reset readySmoothPathList
	mapManager.pathFindFlag = false
	mapManager.readySmoothPathList = mapManager.readySmoothPathList[:0]
	//dir pathList,temporaryImportantIndexOfUselessNode , fina PathList
	mapManager.dirPathList = mapManager.dirPathList[:0]
	mapManager.temporaryImportantIndexOfUselessNode = mapManager.temporaryImportantIndexOfUselessNode[:0]
	mapManager.afterUseLessPathList = mapManager.afterUseLessPathList[:0]
	mapManager.repeatAfterUseLessPathList = mapManager.repeatAfterUseLessPathList[:0]
	mapManager.FinalPathList = mapManager.FinalPathList[:0]
	//important inflectionIndex and unimportant inflectionIndex
	mapManager.importantFinalInflectionIndex = mapManager.importantFinalInflectionIndex[:0]
	mapManager.allFinalInflectionIndex = mapManager.allFinalInflectionIndex[:0]
	//about deleteUselessNode method
	clear(mapManager.recordSaveIndexSlice)
	mapManager.sortSlice = mapManager.sortSlice[:0]
	clear(mapManager.temRecordSaveIndexSlice)
	mapManager.temSortSlice = mapManager.temSortSlice[:0]
	mapManager.temRecordNodeSlice = mapManager.temRecordNodeSlice[:0]
	mapManager.deleteTime = 0
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

// reset Node From GameAbFile
func (mapManager *MapManager) resetNodeFromMapData(x, y int16) {
	mapManager.mapData[x][y].f = 0
	mapManager.mapData[x][y].g = 0
	mapManager.mapData[x][y].h = 0
	mapManager.mapData[x][y].father = nil
	mapManager.mapData[x][y].open = false
	mapManager.mapData[x][y].closed = false
	mapManager.printPathList[x][y] = "."
}

func (mapManager *MapManager) smoothPath() {
	if len(mapManager.readySmoothPathList) <= 2 {
		fmt.Println("just 2 node,not need smooth path")
		mapManager.afterUseLessPathList = append(mapManager.afterUseLessPathList, mapManager.readySmoothPathList...)
		return
	}
	//-----------------------------------------------------delete useless node(node >= 3)-----------------------------------------------------
	mapManager.PrintTime.UseLessCost = time.Now().UnixMicro()
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
	//---------------------------------------------------------双指针减少冗余拐点---------------------------------------------------------
	//对去重后的大部分拐点进行第一次双指针减少
	mapManager.deleteUseLessNodeForSecond()
	//存储一下还没有添加2个最远优拐点的AfterUseLessPathList到repeatAfterUseLessPathList
	mapManager.repeatAfterUseLessPathList = append(mapManager.repeatAfterUseLessPathList, mapManager.FinalPathList...)
	//分别找离起点和终点最远的第二优拐点 并减少2边拐点
	mapManager.findSecondNode()
	mapManager.PrintTime.UseLessCost = time.Now().UnixMicro() - mapManager.PrintTime.UseLessCost
	//如果减少之后的FinalPathList剩余拐点仍然大于18个 那么直接返回所有路径点，不需要set Combinations Index了
	if len(mapManager.FinalPathList) >= 18 {
		for i := 0; i < len(mapManager.FinalPathList); i++ {
			//直接把FinalPathList当做最终路径，因为太多拐点了，添加索引进SmoothFinalIndex
			mapManager.SmoothValType.SmoothFinalIndex = append(mapManager.SmoothValType.SmoothFinalIndex, i)
		}
		return
	} else {
		//-----------------------------------------------------set Combinations Index And get Combinations-----------------------------------------------------
		//找到所有拐点可能
		mapManager.setAllCombinationsFinalIndexMap()
		//mapManager.SmoothValType.allCombinationsFinalIndexMap[23] = [][]int{
		//	{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22},
		//}
		mapManager.PrintTime.SetCombinationNodeCost = time.Now().UnixMicro() - mapManager.PrintTime.SetCombinationNodeCost
		//
		//-----------------------------------------------------recursive Obstacle Check-----------------------------------------------------
		mapManager.PrintTime.SmoothBestWay = time.Now().UnixMicro()
		//smoothBestWay
		mapManager.smoothBestWay()
		mapManager.PrintTime.SmoothBestWay = time.Now().UnixMicro() - mapManager.PrintTime.SmoothBestWay
		//for index, val := range mapManager.FinalPathList {
		//	mapManager.printPathList[val.X][val.Y] = strconv.Itoa(index)
		//}
	}
}

// x1x2是首尾点 x3是中间点
func (mapManager *MapManager) isPointOnLineBetweenOptimized(x1, y1, x2, y2, x3, y3 int) bool {
	// 判断是否共线
	if (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1) != 0 {
		return false
	}

	// 判断是否位于两点之间 不需要等于 因为等于的话 相当于同一个点 没必要
	if (x3 > x1 && x3 < x2) || (x3 > x2 && x3 < x1) {
		if (y3 > y1 && y3 < y2) || (y3 > y2 && y3 < y1) {
			return true
		}
	}
	return false
}

func (mapManager *MapManager) setAllCombinationsFinalIndexMap() {
	//-----------------------------------------------------set Combinations Index And get Combinations-----------------------------------------------------
	mapManager.PrintTime.SetCombinationNodeCost = time.Now().UnixMicro()
	//添加除了头尾的索引到allFinalInflectionIndex
	for i := 1; i < len(mapManager.FinalPathList)-1; i++ {
		//添加除了首尾的所有点进allFinalInflectionIndex
		mapManager.allFinalInflectionIndex = append(mapManager.allFinalInflectionIndex, i)
	}
	//like [1] = [[0],[1]] || [2] = [[0,1]]
	//lenOfAllFinalInflectionIndex := 0
	//if len(mapManager.allFinalInflectionIndex) > 20 {
	//	lenOfAllFinalInflectionIndex = 8
	//	for i := 1; i <= lenOfAllFinalInflectionIndex; i++ {
	//		mapManager.SmoothValType.allCombinationsFinalIndexMap[i] = mapManager.generateCombinations(len(mapManager.allFinalInflectionIndex)-1, i)
	//	}
	//} else {
	//	for i := 1; i <= len(mapManager.allFinalInflectionIndex); i++ {
	//		mapManager.SmoothValType.allCombinationsFinalIndexMap[i] = mapManager.generateCombinations(len(mapManager.allFinalInflectionIndex)-1, i)
	//	}
	//}
	for i := 1; i <= len(mapManager.allFinalInflectionIndex); i++ {
		mapManager.SmoothValType.allCombinationsFinalIndexMap[i] = mapManager.generateCombinations(len(mapManager.allFinalInflectionIndex)-1, i)
	}
}

func (mapManager *MapManager) smoothBestWay() {
	//success H temporary h
	h, H := 0.0, 0.0
	//successCombinationsPath false
	successCombinationsPath := false
	//for i := 1; i <= len(mapManager.SmoothValType.allCombinationsFinalIndexMap); i++ {
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
func (mapManager *MapManager) checkCombinationsPath(combinations []int, startIndex, endIndex int) (bool, float64) {
	h := 0.0
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
				H = mapManager.euclideanDistance(
					mapManager.FinalPathList[current].X, mapManager.FinalPathList[current].Y,
					mapManager.FinalPathList[combinationsIndexOfFinalPath].X, mapManager.FinalPathList[combinationsIndexOfFinalPath].Y)
				//set map  map[int]map[int]int 1 to 1 to 1
				mapManager.SmoothValType.successAllObstacleFinalIndexHValMap[current] = make(map[int]float64, 1)
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
			H = mapManager.euclideanDistance(
				mapManager.FinalPathList[current].X, mapManager.FinalPathList[current].Y,
				mapManager.FinalPathList[endIndex].X, mapManager.FinalPathList[endIndex].Y)
			//set map  map[int]map[int]int 1 to 1 to 1
			mapManager.SmoothValType.successAllObstacleFinalIndexHValMap[current] = make(map[int]float64, 1)
			mapManager.SmoothValType.successAllObstacleFinalIndexHValMap[current][endIndex] = H //!!! H
			//
			h += H
			return true, h
		} else {
			//if obstacleJudge filed , means no way
			return false, 0
		}
	}
}

// return ok
func (mapManager *MapManager) getAllObstacleFinalIndexHValMap(startIndex, endIndex int) (bool, float64) {
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

func (mapManager *MapManager) getSmaller(a, b int16) int16 {
	if a < b {
		return a
	} else {
		return b
	}
}
func (mapManager *MapManager) getLarger(a, b int16) int16 {
	if a > b {
		return a
	} else {
		return b
	}
}

// Generate Combinations --n:len k:want Combinations
func (mapManager *MapManager) generateCombinations(n, k int) [][]int {
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
			//if ip {
			//	for index, val := range stack {
			//		combination[index] = mapManager.importantFinalInflectionIndex[val]
			//	}
			//} else {
			//	for index, val := range stack {
			//		combination[index] = mapManager.allFinalInflectionIndex[val]
			//	}
			//}
			for index, val := range stack {
				combination[index] = mapManager.allFinalInflectionIndex[val]
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

func (mapManager *MapManager) checkImportantInflectionIndex(x, y int16) bool {
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

func (mapManager *MapManager) pathFind(x1, y1, x2, y2 int16) {
	var node *Node
	//heap.Init(&mapManager.openList)
	//start path find:8 direction
	offsetX := int16(0)
	offsetY := int16(0)
	var f, g, h float32 = 0, 0, 0
	for {
		for index, offset := range rangeOffset {
			//may eight times
			//{0, 1}right 		{0, -1}left 	 	 {-1, 0}up 	     	 {1, 0}down
			//{-1, 1}right up 	{1, 1}right down 	 {-1, -1}left up 	 {1, -1}left down
			f, g, h = 0, 0, 0
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
				//
				//if k=-1/1,judge 2 obstacle around of 4 status,if have 1 obstacle,continue
				if index >= 4 {
					//判断4个八向的斜向 如果经过有1个障碍 那么不是偏移点 直接continue下一个
					switch index {
					case 4:
						//! OFFSET
						//F !
						if !mapManager.checkBoundaryAndObstacle(offsetX, offsetY-1) {
							continue
						}
						if !mapManager.checkBoundaryAndObstacle(offsetX+1, offsetY) {
							continue
						}
					case 5:
						//F !
						//! OFFSET
						if !mapManager.checkBoundaryAndObstacle(offsetX-1, offsetY) {
							continue
						}
						if !mapManager.checkBoundaryAndObstacle(offsetX, offsetY-1) {
							continue
						}
					case 6:
						//OFFSET !
						//!      F
						if !mapManager.checkBoundaryAndObstacle(offsetX, offsetY+1) {
							continue
						}
						if !mapManager.checkBoundaryAndObstacle(offsetX+1, offsetY) {
							continue
						}
					case 7:
						//!      F
						//OFFSET !
						if !mapManager.checkBoundaryAndObstacle(offsetX-1, offsetY) {
							continue
						}
						if !mapManager.checkBoundaryAndObstacle(offsetX, offsetY+1) {
							continue
						}
					}
					//1.41
					g += math.Sqrt2
				} else {
					//4个八向的平向 不用判断直接加代价就行
					g += 1
				}
				//------16向分割
				//if index >= 4 {
				//	//判断8个十六向 如果经过有1个障碍 那么不是偏移点 直接continue下一个
				//	switch index {
				//	//感叹号是障碍
				//	case 4:
				//		//OFFSET !	   *
				//		//*      !     F
				//		if !mapManager.checkBoundaryAndObstacle(offsetX, offsetY+1) {
				//			continue
				//		}
				//		if !mapManager.checkBoundaryAndObstacle(offsetX+1, offsetY+1) {
				//			continue
				//		}
				//	case 5:
				//		//OFFSET *
				//		//!	 	 !
				//		//* 	 F
				//		if !mapManager.checkBoundaryAndObstacle(offsetX+1, offsetY) {
				//			continue
				//		}
				//		if !mapManager.checkBoundaryAndObstacle(offsetX+1, offsetY+1) {
				//			continue
				//		}
				//	case 6:
				//		//*   OFFSET
				//		//!      !
				//		//F		 *
				//		if !mapManager.checkBoundaryAndObstacle(offsetX+1, offsetY-1) {
				//			continue
				//		}
				//		if !mapManager.checkBoundaryAndObstacle(offsetX+1, offsetY) {
				//			continue
				//		}
				//	case 7:
				//		//*  ! OFFSET
				//		//F  !	 *
				//		if !mapManager.checkBoundaryAndObstacle(offsetX, offsetY-1) {
				//			continue
				//		}
				//		if !mapManager.checkBoundaryAndObstacle(offsetX+1, offsetY-1) {
				//			continue
				//		}
				//	case 8:
				//		//F  !   *
				//		//*  ! OFFSET
				//		if !mapManager.checkBoundaryAndObstacle(offsetX-1, offsetY-1) {
				//			continue
				//		}
				//		if !mapManager.checkBoundaryAndObstacle(offsetX, offsetY-1) {
				//			continue
				//		}
				//	case 9:
				//		//F      *
				//		//!      !
				//		//*	   OFFSET
				//		if !mapManager.checkBoundaryAndObstacle(offsetX-1, offsetY-1) {
				//			continue
				//		}
				//		if !mapManager.checkBoundaryAndObstacle(offsetX-1, offsetY) {
				//			continue
				//		}
				//	case 10:
				//		//  *      F
				//		//  !      !
				//		//OFFSET   *
				//		if !mapManager.checkBoundaryAndObstacle(offsetX-1, offsetY) {
				//			continue
				//		}
				//		if !mapManager.checkBoundaryAndObstacle(offsetX-1, offsetY+1) {
				//			continue
				//		}
				//	case 11:
				//		//  *  	  !     F
				//		//OFFSET  !   	*
				//		if !mapManager.checkBoundaryAndObstacle(offsetX, offsetY+1) {
				//			continue
				//		}
				//		if !mapManager.checkBoundaryAndObstacle(offsetX-1, offsetY+1) {
				//			continue
				//		}
				//	}
				//	//sqrt5 = 2.236
				//	g += sqrt5
				//} else {
				//	g += 1
				//}
				//g += 1
				//判断偏移点是不是障碍点
				if mapManager.mapData[offsetX][offsetY].nodeType == 0 {
					//g from this center node(offset father)
					g += mapManager.mapData[x1][y1].g
					//h from this offset node to end
					//h = float32(math.Abs(float64(x2-offsetX)) + math.Abs(float64(y2-offsetY)))
					h = float32(mapManager.diagonalDistance(x2, y2, offsetX, offsetY))
					h += float32(mapManager.mapData[offsetX][offsetY].nodeRoadNum)
					//cross := mapManager.CalculateCross(
					//	mapManager.mapData[offsetX][offsetY],
					//	mapManager.mapData[mapManager.startNodeX][mapManager.startNodeY],
					//	mapManager.mapData[x2][y2])
					//weight := mapManager.CalculateWeight(
					//	mapManager.mapData[offsetX][offsetY],
					//	mapManager.mapData[mapManager.startNodeX][mapManager.startNodeY],
					//	mapManager.mapData[x2][y2], 0.7)
					// 调整路径反向偏离程度
					//deviation := mapManager.AdjustPathDeviation(cross, weight, 0.1)
					//h = h + cross*0.001
					//h = h + deviation
					//calculate f = g + h
					//h越大 代表g越不重要，说明越靠近终点的点 就是这个路径上的点，越贪心越快
					f = g + 1*h
					//save f,g,h
					mapManager.mapData[offsetX][offsetY].f = f
					mapManager.mapData[offsetX][offsetY].g = g
					mapManager.mapData[offsetX][offsetY].h = h
					//save father node
					mapManager.mapData[offsetX][offsetY].father = mapManager.mapData[x1][y1]
					//Push Push Push Push
					Push(&mapManager.openList, mapManager.mapData[offsetX][offsetY])
					mapManager.mapData[offsetX][offsetY].open = true
					//查看多少节点被检测了
					//mapManager.judgeList = append(mapManager.judgeList, mapManager.mapData[offsetX][offsetY])
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
								reverseSlice(&mapManager.readySmoothPathList)
								//mapManager.pathList assign to readySmoothPathList
								//copy(mapManager.readySmoothPathList, mapManager.pathList)
								//var time2 = time.Now().UnixMicro()
								//fmt.Println("reverseSlice time 微秒: ", time2-time1)
								//
								for _, val := range mapManager.readySmoothPathList {
									//fmt.Println(index, " -> ", " x: ", val.x, " y: ", val.y, " nodeType: ", val.nodeType)
									mapManager.printPathList[val.X][val.Y] = "*"
								}
								mapManager.printPathList[mapManager.startNodeX][mapManager.startNodeY] = "S"
								mapManager.printPathList[x2][y2] = "E"
								//set pathFindFlag true,success find path
								mapManager.pathFindFlag = true
								return
							}
						}
					}
				}
			}
		}
		if len(mapManager.openList) > 0 {
			//add latest node into closeList from openList,remove latest node from openList,set flag
			//Pop Pop Pop Pop
			node = Pop(&mapManager.openList)
			node.open = false
			node.closed = true
			mapManager.closeList = append(mapManager.closeList, node)
			//recursion pathFind method,start is closeList latest node
			x1 = mapManager.closeList[len(mapManager.closeList)-1].X
			y1 = mapManager.closeList[len(mapManager.closeList)-1].Y
			continue
			//mapManager.pathFind(mapManager.closeList[len(mapManager.closeList)-1].X, mapManager.closeList[len(mapManager.closeList)-1].Y, x2, y2)
		} else {
			//if not node in openList now
			fmt.Println("openList is empty, no way find!")
			return
		}
	}
}

// CalculateWeight 计算当前节点的权重因子
// startPercentage 是一个比例值，表示从起点开始的某个范围内，权重因子会逐渐减少。
// 例如，如果 startPercentage = 0.2，表示在起点到终点的总距离的前 20% 范围内，权重因子会从 1 逐渐减少到 0。
// 超出这个范围后，权重因子直接设为 0。
func (mapManager *MapManager) CalculateWeight(node, start, end *Node, startPercentage float64) float64 {
	// 计算当前节点到起点的距离 起点 当前
	distToStart := math.Sqrt(math.Pow(float64(node.X-start.X), 2) + math.Pow(float64(node.Y-start.Y), 2))

	// 计算起点到终点的总距离 起点 终点
	totalDist := math.Sqrt(math.Pow(float64(end.X-start.X), 2) + math.Pow(float64(end.Y-start.Y), 2))

	// 计算当前节点的位置比例 t
	t := distToStart / totalDist

	// 动态调整权重因子
	weight := 0.0
	if t <= startPercentage {
		// 在起点部分范围内，权重从 1 逐渐减少到 0
		weight = 1 - t/startPercentage
	} else {
		// 超出起点部分范围，权重为 0
		weight = t/startPercentage - 1
	}

	return weight
}

// CalculateCross 计算偏移量 cross
func (mapManager *MapManager) CalculateCross(node, start, end *Node) float64 {
	// 计算 dx1, dy1（当前节点到终点的距离）
	dx1 := math.Abs(float64(node.X - end.X))
	dy1 := math.Abs(float64(node.Y - end.Y))

	// 计算 dx2, dy2（起点到终点的距离）
	dx2 := math.Abs(float64(start.X - end.X))
	dy2 := math.Abs(float64(start.Y - end.Y))

	// 计算叉积 cross
	cross := math.Abs(dx1*dy2 - dx2*dy1)

	return cross
}

// AdjustPathDeviation 调整路径偏离程度
func (mapManager *MapManager) AdjustPathDeviation(cross, weight, deviationFactor float64) float64 {
	// 反转偏移量的影响：cross 越大，路径越偏离连线
	deviation := cross * weight * deviationFactor
	return deviation
}

// 最精确的移动八向对角线距离公式 h(n)<=真实h(n)
func (mapManager *MapManager) diagonalDistance(startX, startY, endX, endY int16) float64 {
	dx := math.Abs(float64(startX - endX))
	dy := math.Abs(float64(startY - endY))
	if dx == 0 || dy == 0 {
		return dx + dy
	} else {
		// 对角线移动
		return dx + dy + (math.Sqrt2-2)*(min(dx, dy))
	}
}

// 计算曼哈顿距离
func (mapManager *MapManager) manhattanDistance(x1, y1, x2, y2 int16) float64 {
	return math.Abs(float64(x2-x1)) + math.Abs(float64(y2-y1))
}

// 欧几里得距离公式
func (mapManager *MapManager) euclideanDistance(startX, startY, endX, endY int16) float64 {
	dx := float64(startX - endX)
	dy := float64(startY - endY)
	return math.Sqrt(dx*dx + dy*dy)
}

// 欧几里得距离公式
func (mapManager *MapManager) euclideanDistanceForPathFind(startX, startY, endX, endY int16) float64 {
	dx := float64(startX - endX)
	dy := float64(startY - endY)
	if mapManager.obstacleJudge(mapManager.mapData[startX][startY], mapManager.mapData[endX][endY]) {
		//如果有障碍 尝试放大代价
		return math.Sqrt(dx*dx+dy*dy) * 2
	} else {
		return math.Sqrt(dx*dx + dy*dy)
	}

}

// 计算切比雪夫距离
func (mapManager *MapManager) chebyshevDistance(x1, y1, x2, y2 int16) float64 {
	return math.Max(math.Abs(float64(x2-x1)), math.Abs(float64(y2-y1)))
}

// 混合启发式函数：结合曼哈顿距离和切比雪夫距离
func (mapManager *MapManager) mixedHeuristic(startX, startY, endX, endY int16, k float64) float64 {
	dx := math.Abs(float64(startX - endX))
	dy := math.Abs(float64(startY - endY))
	// 混合启发式：max(dx, dy) + 0.5 * min(dx, dy)
	return math.Max(dx, dy) + k*math.Min(dx, dy)
}

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

func (mapManager *MapManager) checkNodeIsObstacle(x, y int16) bool {
	return mapManager.mapData[x][y].nodeType == 1
}

func (mapManager *MapManager) checkYKXBForY(x int16, realY float64) bool {
	//left right
	floor := math.Floor(realY)
	ceil := math.Ceil(realY)
	if ceil-realY <= 0.01 {
		//close is ceil,check 4 obstacle
		if mapManager.checkNodeIsObstacle(x, int16(ceil)) ||
			mapManager.checkNodeIsObstacle(x-1, int16(ceil)) ||
			mapManager.checkNodeIsObstacle(x, int16(ceil)-1) ||
			mapManager.checkNodeIsObstacle(x-1, int16(ceil)-1) {
			return false
		}
	} else if realY-floor <= 0.01 {
		//close is floor,check 4 obstacle
		if mapManager.checkNodeIsObstacle(x, int16(floor)) ||
			mapManager.checkNodeIsObstacle(x-1, int16(floor)) ||
			mapManager.checkNodeIsObstacle(x, int16(floor)-1) ||
			mapManager.checkNodeIsObstacle(x-1, int16(floor)-1) {
			return false
		}
	} else {
		//close is center,check 2 obstacle(left right)
		if mapManager.checkNodeIsObstacle(x, int16(realY)) ||
			mapManager.checkNodeIsObstacle(x-1, int16(realY)) {
			return false
		}
	}
	return true
}

func (mapManager *MapManager) checkYKXBForX(y int16, realX float64) bool {
	//left right
	floor := math.Floor(realX)
	ceil := math.Ceil(realX)
	if ceil-realX <= 0.01 {
		//close is ceil,check 4 obstacle
		if mapManager.checkNodeIsObstacle(int16(ceil), y) ||
			mapManager.checkNodeIsObstacle(int16(ceil)-1, y) ||
			mapManager.checkNodeIsObstacle(int16(ceil), y-1) ||
			mapManager.checkNodeIsObstacle(int16(ceil)-1, y-1) {
			return false
		}
	} else if realX-floor <= 0.01 {
		//close is floor,check 4 obstacle
		if mapManager.checkNodeIsObstacle(int16(floor), y) ||
			mapManager.checkNodeIsObstacle(int16(floor)-1, y) ||
			mapManager.checkNodeIsObstacle(int16(floor), y-1) ||
			mapManager.checkNodeIsObstacle(int16(floor)-1, y-1) {
			return false
		}
	} else {
		//close is center,check 2 obstacle(up down)
		if mapManager.checkNodeIsObstacle(int16(realX), y) ||
			mapManager.checkNodeIsObstacle(int16(realX), y-1) {
			return false
		}
	}
	return true
}

// true是正常0点 不是障碍或者边界外
func (mapManager *MapManager) checkBoundaryAndObstacle(x, y int16) bool {
	if x >= 0 && y >= 0 && x < mapManager.rows && y < mapManager.cols {
		return mapManager.mapData[x][y].nodeType != 1
	}
	//
	return false
}

// 前提 afterUseLessPathList长度 >= 3,并且afterUseLessPathList里面的拐点的首尾点一定不相连，不会出现recordSaveIndexSlice存储起点索引0的情况
// 作用：recordSaveIndexSlice保留 除了！ 除了！ 除了！首尾索引！的有用拐点
// 部分递归通过双指针找到最短的路径拐点集合
func (mapManager *MapManager) deleteUseLessNodeForSecond() {
	startIndex := 0
	endIndex := 1
	//记录要保留的点 这是存储的afterUseLessPathList的索引,用切片可能会出现重复，直接用Map的key唯一性避免重复
	//但是map的key是无序遍历的，所以再添加回切片，然后对切片进行排序
	//mapManager.temRecordSaveIndexSlice := make(map[int]interface{}, 64)
	//mapManager.temSortSlice := make([]int, 0, 64)
	//mapManager.temRecordNodeSlice := make([]*Node, 0, 64)
	//mapManager.deleteTime := 0
	mapManager.deleteTime++
	//如果是第一次 那么对afterUseLessPathList进行双指针
	if mapManager.deleteTime == 1 {
		mapManager.temRecordNodeSlice = append(mapManager.temRecordNodeSlice, mapManager.afterUseLessPathList...)
	}
	//归0一下map和sortSlice准备给双指针存储中间拐点索引使用
	clear(mapManager.recordSaveIndexSlice)
	mapManager.sortSlice = mapManager.sortSlice[:0]
	//双指针找到可能比temRecordNodeSlice更少的中间拐点索引点并存储
	for {
		if endIndex < len(mapManager.temRecordNodeSlice)-1 {
			//如果当前双指针可以相连(索引相邻 或者 判断成功)
			if endIndex-startIndex == 1 || mapManager.obstacleJudge(mapManager.temRecordNodeSlice[startIndex], mapManager.temRecordNodeSlice[endIndex]) {
				//继续判断下一个 有可能这种情况，0到终点的前一个都可以，0到终点不行，如果这时候endIndex是终点的前一个
				endIndex++
				//现在endIndex++过后 endIndex是终点索引，那么就要存储当前判断的startIndex和 索引endIndex-1 然后直接退出
				if endIndex == len(mapManager.temRecordNodeSlice)-1 {
					mapManager.recordSaveIndexSlice[startIndex] = nil
					mapManager.recordSaveIndexSlice[endIndex-1] = nil
					break
				}
			} else {
				//如果当前二指针不能相连，那么二指针的上一个点是要保留的索引点
				mapManager.recordSaveIndexSlice[endIndex-1] = nil
				//这个时候 把一指针放到这个保留的索引点，相当于新的起点，二指针现在已经是新起点的下一个
				startIndex = endIndex - 1
			}
		}
		//如果endIndex 二指针已经到终点了，那么只需要对一指针不断推进判断与终点的连接
		if endIndex == len(mapManager.temRecordNodeSlice)-1 {
			//假如第一次进来，一指针直接与二指针也就是终点可以连接，那么一指针就是最后一个要保留的有用拐点，直接退出循环
			if endIndex-startIndex == 1 || mapManager.obstacleJudge(mapManager.temRecordNodeSlice[startIndex], mapManager.temRecordNodeSlice[endIndex]) {
				//如果是终点前一个或者直接与终点相连，那么第一startIndex指针就是最后一个有用拐点
				//startIndex不可能是起点 因为如果起点直接与终点相连 那么都不会开启寻路 更不会进入这个方法
				mapManager.recordSaveIndexSlice[startIndex] = nil
				break
			} else {
				//如果一指针不可以与二指针连接，那么一指针也是要保留的有用拐点，保留之后平移一指针到下一位继续与终点二指针判断，直到成功为止
				//因为一指针如果是最后一个的前一个与终点判断一定会成功，所以一指针不用判断边界条件
				mapManager.recordSaveIndexSlice[startIndex] = nil
				startIndex++
				//记得continue 防止我第二指针都到终点了，第一startIndex指针还走下面的逻辑，不可能走的
				continue
			}
		}
	}
	//如果是第一次双指针去除 那么先赋值给FinalPathList方便对比
	if len(mapManager.FinalPathList) == 0 {
		//清空并重设FinalPathList
		mapManager.resetFinalPathListForDeleteUseLessNode()
		//因为是第一次 让temRecordNodeSlice先等于FinalPathList
		mapManager.temRecordNodeSlice = mapManager.temRecordNodeSlice[:0]
		mapManager.temRecordNodeSlice = append(mapManager.temRecordNodeSlice, mapManager.FinalPathList...)
		//递归第2次
		mapManager.deleteUseLessNodeForSecond()
	} else {
		//如果已经是第二次 也就是有长度了
		//先存储旧长度
		lenLast := len(mapManager.FinalPathList)
		//清空并重设FinalPathList 这时候FinalPathList的长度有可能比原来 少 或者 相等
		mapManager.resetFinalPathListForDeleteUseLessNode()
		//对比长度有没有变化 如果没变化 代表已经最大限度双指针优化了 return退出递归
		if len(mapManager.FinalPathList) == lenLast {
			return
		} else if len(mapManager.FinalPathList) < lenLast {
			//如果长度变短了 那么继续递归
			//注意 递归前 temRecordNodeSlice重置换为当前减少后的FinalPathList
			mapManager.temRecordNodeSlice = mapManager.temRecordNodeSlice[:0]
			mapManager.temRecordNodeSlice = append(mapManager.temRecordNodeSlice, mapManager.FinalPathList...)
			//递归第n次
			mapManager.deleteUseLessNodeForSecond()
		}
	}

}

// 清空FinalPathList
// 重设temRecordNodeSlice头尾索引和中间部分索引点给FinalPathList
// 用recordSaveIndexSlice的map，遍历出无序key,然后用sortSlice排序无序key，key是temRecordNodeSlice的索引，然后append进FinalPathList
// 这一步之后 FinalPathList的点有可能比temRecordNodeSlice 少 或者 相等
func (mapManager *MapManager) resetFinalPathListForDeleteUseLessNode() {
	//清空FinalPathList
	mapManager.FinalPathList = mapManager.FinalPathList[:0]
	//添加起点
	mapManager.FinalPathList = append(mapManager.FinalPathList, mapManager.temRecordNodeSlice[0])
	//添加recordSaveIndexSlice的key,key就是afterUseLessPathList的索引
	for key, _ := range mapManager.recordSaveIndexSlice {
		//记得判断一下key索引有可能是起点的情况 如果是起点就不添加，因为0 1 2三点都可以的话，0 和 2不成功 0会被添加进key
		if key != 0 {
			mapManager.sortSlice = append(mapManager.sortSlice, key)
		}
	}
	//从小到大
	sort.Ints(mapManager.sortSlice)
	for i := 0; i < len(mapManager.sortSlice); i++ {
		//mapManager.afterUseLessPathList[a] a就是recordSaveIndexSlice[i]索引值 sortSlice[i]不会出现起点0索引 前面key遍历的时候已经过滤掉了
		mapManager.FinalPathList = append(mapManager.FinalPathList, mapManager.temRecordNodeSlice[mapManager.sortSlice[i]])
	}
	//添加终点
	mapManager.FinalPathList = append(mapManager.FinalPathList, mapManager.temRecordNodeSlice[len(mapManager.temRecordNodeSlice)-1])
}

// 这个不用那么麻烦，直接从readySmoothPathNode的终点开始往上找到最后第一个能与终点直接相连的点，也就是说他是离终点最远的有效拐点
// 因为上一步已经从起点开始做类似的操作了，由于A星贪心算法的尾部局限性，尾巴部分的路径可能会撞到墙然后走直线造成路径不是意义上的最短，所以要进行找倒数第二优拐点的操作
// 明确一下 这个倒数第二优拐点一定是readySmoothPathList(经过普通直线去重之后)的2个拐点之间的点
// 找到这个拐点之后，删掉这个拐点之后的所有拐点，然后把这个拐点插入到FinalPathList的终点之前，这样就是最优的路径了
// 边界检查：上一步起点开始去重可能FinalPathList最低3个点
// 检查离终点最远的拐点并添加 应该从起点的下一个点开始检查

// 记得改回来！！！！ 目前只找终点的优点 起点屏蔽了
func (mapManager *MapManager) findSecondNode() {
	var secondToStartNode *Node = nil
	var secondToLastNode *Node = nil
	secondToStartNodeIndexInReadySmoothPathList := 0
	secondToLastNodeIndexInReadySmoothPathList := 0
	//找离起点最远优拐点
	//开始点是从终点的前一个点开始 因为是最远点 起点0不遍历
	for i := len(mapManager.readySmoothPathList) - 2; i >= 1; i-- {
		//判断直线障碍 如果成功相连
		if mapManager.obstacleJudge(
			mapManager.readySmoothPathList[0],
			mapManager.readySmoothPathList[i]) {
			secondToStartNode = mapManager.readySmoothPathList[i]
			secondToStartNodeIndexInReadySmoothPathList = i
			break
		}
	}
	//找离终点最远优拐点
	//开始点是从起点的下一个点开始 因为是最远点 终点不遍历
	for i := 1; i < len(mapManager.readySmoothPathList)-1; i++ {
		//判断直线障碍 如果成功相连
		if mapManager.obstacleJudge(
			mapManager.readySmoothPathList[len(mapManager.readySmoothPathList)-1],
			mapManager.readySmoothPathList[i]) {
			secondToLastNode = mapManager.readySmoothPathList[i]
			secondToLastNodeIndexInReadySmoothPathList = i
			break
		}
	}
	if secondToStartNode == nil && secondToLastNode == nil {
		return
	} else if secondToStartNode != nil && secondToLastNode == nil {
		// 如果只有离起点的最远优拐点
		mapManager.insertForStartNode(secondToStartNode, secondToStartNodeIndexInReadySmoothPathList)
	} else if secondToStartNode == nil && secondToLastNode != nil {
		// 如果只有离终点的最远优拐点 插入离终点的最远优拐点
		mapManager.insertForLastNode(secondToLastNode, secondToLastNodeIndexInReadySmoothPathList)
	} else {
		//如果2个点都存在
		if secondToStartNodeIndexInReadySmoothPathList < secondToLastNodeIndexInReadySmoothPathList {
			//如果如果离起点最远优拐点 在离终点最远优拐点 的前面 那么就是普遍情况 各自删除
			//插入离终点的最远优拐点
			mapManager.insertForStartNode(secondToStartNode, secondToStartNodeIndexInReadySmoothPathList)
			mapManager.insertForLastNode(secondToLastNode, secondToLastNodeIndexInReadySmoothPathList)
		} else if secondToStartNodeIndexInReadySmoothPathList > secondToLastNodeIndexInReadySmoothPathList {
			//如果离起点最远优拐点 在离终点最远优拐点 的后面 那么就是这4个点
			//添加顺序：起点 LastNodeIndex StartNodeIndex 终点
			mapManager.FinalPathList = mapManager.FinalPathList[:0]
			mapManager.FinalPathList = append(mapManager.FinalPathList,
				mapManager.readySmoothPathList[0],
				mapManager.readySmoothPathList[secondToLastNodeIndexInReadySmoothPathList],
				mapManager.readySmoothPathList[secondToStartNodeIndexInReadySmoothPathList],
				mapManager.readySmoothPathList[len(mapManager.readySmoothPathList)-1],
			)
		} else if secondToStartNodeIndexInReadySmoothPathList == secondToLastNodeIndexInReadySmoothPathList {
			//如果刚好是一样最优远点 那么就是这3个点
			mapManager.FinalPathList = mapManager.FinalPathList[:0]
			mapManager.FinalPathList = append(mapManager.FinalPathList,
				mapManager.readySmoothPathList[0],
				mapManager.readySmoothPathList[secondToLastNodeIndexInReadySmoothPathList],
				mapManager.readySmoothPathList[len(mapManager.readySmoothPathList)-1],
			)
		}
	}
}

// insert For StartNode
func (mapManager *MapManager) insertForStartNode(insertNode *Node, insertIndex int) {
	//这一步是想把之前的倒数第二优拐点 插入 插入 到同线的FinalPathList拐点里面
	//双指针索引从后往前 索引来自FinalPathList(已经双指针从头开始优化过一遍的拐点)
	//注意一下 这个双指针索引是FinalPathList里面的有效拐点索引
	//startIndex是倒数第二个索引 endIndex是倒数第一 然后这2个指针一直往前移动 直到符合 或者 startIndex到第一个
	//FinalPathList最低是3个点 0 拐点 终点
	startIndex := len(mapManager.FinalPathList) - 2
	endIndex := len(mapManager.FinalPathList) - 1
	for {
		//startIndex最后到1判断完就退出了 不会对0 1判断 比如 0 1 2 ,startIndex为1的时候进行1 2最后一次判断
		if startIndex <= 0 {
			break
		}
		if insertNode == mapManager.FinalPathList[startIndex] {
			//左边 清左边的点 所以还是startIndex
			mapManager.insertSliceElementAndDeletePart1(&mapManager.FinalPathList, insertNode, startIndex, true)
			break
		}
		if insertNode == mapManager.FinalPathList[endIndex] {
			mapManager.insertSliceElementAndDeletePart1(&mapManager.FinalPathList, insertNode, endIndex, true)
			break
		}
		//isBetweenFinalPathInReadySmoothPath方法判断secondToLastNode这个点是否在ReadySmoothPath中处于索引之间 这里一定一定要用ReadySmoothPath来判断 不能用afterUseLessPathList
		//对于起点最远优拐点来说 如果处于2者之间 且 与下一个原有拐点能直线障碍成功 才能正确删除路线前面的多余拐点 它才是合格的起点最远优拐点
		if mapManager.isBetweenFinalPathInReadySmoothPath(mapManager.FinalPathList[startIndex], mapManager.FinalPathList[endIndex], insertNode, insertIndex) &&
			mapManager.obstacleJudge(mapManager.FinalPathList[endIndex], insertNode) {
			//如果是 则他是离终点的第二优拐点 插入FinalPathList 这个方法会将元素插入 然后 删除这个元素之后的所有元素
			mapManager.insertSliceElementAndDeletePart1(&mapManager.FinalPathList, insertNode, endIndex, false)
		} else {
			//不断前移startIndex和endIndex
			startIndex--
			endIndex--
		}
	}
}

// slice := []*Node{node1, node2, node3, node4}
// element := newNode
// index := 2
//
// newSlice := mapManager.InsertElementAndTruncateBeforeIndex(slice, element, index)
// fmt.Println(newSlice) // 输出: [起点, newNode, node3, node4]
func (mapManager *MapManager) insertSliceElementAndDeletePart1(slice *[]*Node, element *Node, index int, sameNode bool) {
	if index < 0 || index > len(*slice) {
		panic("index out of range")
	}
	// 1. 创建一个新的切片，容量足够容纳插入后的元素
	newSlice := make([]*Node, 0, len(*slice)+1)
	if sameNode {
		//如果是相同startIndex或者endIndex
		//1.让他等于startIndex和之后的所有元素
		newSlice = append(newSlice, (*slice)[index:]...)
		//2.添加起点
		newSlice = append([]*Node{mapManager.readySmoothPathList[0]}, newSlice...)
	} else {
		// 如果是中间元素 index是endIndex
		// 1. 先插入 endIndex和以后的元素
		newSlice = append(newSlice, (*slice)[index:]...)
		// 5. 添加回起点 和 element 起点被删掉了
		newSlice = append([]*Node{mapManager.readySmoothPathList[0], element}, newSlice...)
	}
	// 最后将新切片赋值给原切片
	*slice = newSlice
}

func (mapManager *MapManager) insertForLastNode(insertNode *Node, insertIndex int) {
	//这一步是想把之前的倒数第二优拐点 插入 插入 到同线的FinalPathList拐点里面
	//双指针索引从后往前 索引来自FinalPathList(已经双指针从头开始优化过一遍的拐点)
	//注意一下 这个双指针索引是FinalPathList里面的有效拐点索引
	//startIndex是倒数第二个索引 endIndex是倒数第一 然后这2个指针一直往前移动 直到符合 或者 startIndex到第一个
	//FinalPathList最低是3个点 0 拐点 终点
	startIndex := 0
	endIndex := 1
	for {
		//endIndex不需要到终点 因为终点和终点前一个点之间不需要判断了 没意义
		if endIndex == len(mapManager.FinalPathList)-1 {
			break
		}
		if insertNode == mapManager.FinalPathList[startIndex] {
			//终点已经在方法里面补上
			mapManager.insertSliceElementAndDeletePart2(&mapManager.FinalPathList, insertNode, startIndex)
			break
		}
		if insertNode == mapManager.FinalPathList[endIndex] {
			//终点已经在方法里面补上
			mapManager.insertSliceElementAndDeletePart2(&mapManager.FinalPathList, insertNode, endIndex)
			break
		}
		//isBetweenFinalPathInReadySmoothPath方法判断secondToLastNode这个点是否在ReadySmoothPath中处于索引之间 这里一定一定要用ReadySmoothPath来判断 不能用afterUseLessPathList
		//对于终点最远优拐点来说 如果处于2者之间 且 与上一个原有拐点能直线障碍成功 才能正确删除路线后面的多余拐点 它才是合格的终点最远优拐点
		if mapManager.isBetweenFinalPathInReadySmoothPath(mapManager.FinalPathList[startIndex], mapManager.FinalPathList[endIndex], insertNode, insertIndex) &&
			mapManager.obstacleJudge(mapManager.FinalPathList[startIndex], insertNode) {
			//如果是 则他是离终点的第二优拐点 插入FinalPathList 这个方法会将元素插入 然后 删除这个元素之后的所有元素 终点已经在方法里面补上
			mapManager.insertSliceElementAndDeletePart2(&mapManager.FinalPathList, insertNode, endIndex)
		} else {
			//不断前移startIndex和endIndex
			startIndex++
			endIndex++
		}
	}
}

// InsertIntSlice 创建一个初始切片
// slice := []int{1, 2, 3, 4, 5}
// 要插入的元素
// element := 10
// 插入的位置（索引）
// index := 2
// 调用方法插入元素
// slice = InsertIntSlice(slice, element, index)
// fmt.Println(slice) // 输出: [1 2 10 3 4 5]
// 然后删除element 10 之前的元素 [index:] [10 3 4 5]<- 也就是说只要element和以后的元素 新索引还是index
// 最后添加回起点 [1 10 3 4 5]
func (mapManager *MapManager) insertSliceElementAndDeletePart2(slice *[]*Node, element *Node, index int) {
	if index < 0 || index > len(*slice) {
		panic("index out of range")
	}
	// 1. 创建一个新的切片，容量足够容纳插入后的元素
	newSlice := make([]*Node, 0, len(*slice)+1)
	// 2. 先插入 part1
	newSlice = append(newSlice, (*slice)[:index]...)
	// 3. 插入目标元素
	newSlice = append(newSlice, element)
	// 4. 然后直接插入终点
	newSlice = append(newSlice, mapManager.readySmoothPathList[len(mapManager.readySmoothPathList)-1])
	// 5. 将新切片赋值给原切片
	*slice = newSlice
}

// 在插入X3之前已经判断过是不是相同FinalPath点了 所以这个方法不用判断
// 判断是否x3这个点是否在ReadySmoothPath中处于x1和x2之间 不需要共线 只需要判断ReadySmoothPath索引的前后关系就行
func (mapManager *MapManager) isBetweenFinalPathInReadySmoothPath(startNode, endNode, midNode *Node, midNodeIndex int) bool {
	//这是给ReadySmoothPath用的
	startIndex, endIndex := 0, 0
	//遍历readySmoothPathList找到这3点的索引 一定能找到 因为midNode就是从readySmoothPathList来的
	for i := 0; i < len(mapManager.readySmoothPathList); i++ {
		if mapManager.readySmoothPathList[i] == startNode {
			startIndex = i
		}
		if mapManager.readySmoothPathList[i] == endNode {
			endIndex = i
		}
	}
	//index是按顺序走的 直接对在readySmoothPathList的索引进行判断就行
	if midNodeIndex > startIndex && midNodeIndex < endIndex {
		return true
	} else {
		return false
	}
}

var sqrt2 = math.Sqrt2
var sqrt5 = math.Sqrt(5)

type printTimeToken struct {
	ResetTimeCost          int64
	PathFindCost           int64
	AllPathFindCost        int64
	UseLessCost            int64
	SetCombinationNodeCost int64
	SmoothBestWay          int64
	EndTime                int64
	StartTime              int64
}

//
//if index >= 4 {
//	//判断8个十六向 如果经过有1个障碍 那么不是偏移点 直接continue下一个
//	switch index {
//	//感叹号是障碍
//	case 4:
//		//OFFSET !	   *
//		//*      !     F
//		if !mapManager.checkBoundaryAndObstacle(offsetX, offsetY+1) {
//			continue
//		}
//		if !mapManager.checkBoundaryAndObstacle(offsetX+1, offsetY+1) {
//			continue
//		}
//	case 5:
//		//OFFSET *
//		//!	 	 !
//		//* 	 F
//		if !mapManager.checkBoundaryAndObstacle(offsetX+1, offsetY) {
//			continue
//		}
//		if !mapManager.checkBoundaryAndObstacle(offsetX+1, offsetY+1) {
//			continue
//		}
//	case 6:
//		//*   OFFSET
//		//!      !
//		//F		 *
//		if !mapManager.checkBoundaryAndObstacle(offsetX+1, offsetY-1) {
//			continue
//		}
//		if !mapManager.checkBoundaryAndObstacle(offsetX+1, offsetY) {
//			continue
//		}
//	case 7:
//		//*  ! OFFSET
//		//F  !	 *
//		if !mapManager.checkBoundaryAndObstacle(offsetX, offsetY-1) {
//			continue
//		}
//		if !mapManager.checkBoundaryAndObstacle(offsetX+1, offsetY-1) {
//			continue
//		}
//	case 8:
//		//F  !   *
//		//*  ! OFFSET
//		if !mapManager.checkBoundaryAndObstacle(offsetX-1, offsetY-1) {
//			continue
//		}
//		if !mapManager.checkBoundaryAndObstacle(offsetX, offsetY-1) {
//			continue
//		}
//	case 9:
//		//F      *
//		//!      !
//		//*	   OFFSET
//		if !mapManager.checkBoundaryAndObstacle(offsetX-1, offsetY-1) {
//			continue
//		}
//		if !mapManager.checkBoundaryAndObstacle(offsetX-1, offsetY) {
//			continue
//		}
//	case 10:
//		//  *      F
//		//  !      !
//		//OFFSET   *
//		if !mapManager.checkBoundaryAndObstacle(offsetX-1, offsetY) {
//			continue
//		}
//		if !mapManager.checkBoundaryAndObstacle(offsetX-1, offsetY+1) {
//			continue
//		}
//	case 11:
//		//  *  	  !     F
//		//OFFSET  !   	*
//		if !mapManager.checkBoundaryAndObstacle(offsetX, offsetY+1) {
//			continue
//		}
//		if !mapManager.checkBoundaryAndObstacle(offsetX-1, offsetY+1) {
//			continue
//		}
//	}
//	//sqrt5 = 2.236
//	g += sqrt5
//} else {
//	//4个八向的平向 不用判断直接加代价就行
//	g += 1
//}
