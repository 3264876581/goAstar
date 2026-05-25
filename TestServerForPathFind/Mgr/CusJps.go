package Cus

import (
	"fmt"
	"math"
	"math/bits"
	"time"

	"github.com/DmitriyVTitov/size"
)

//	arr := [3r][4c]int{
//		{1, 2, 3, 4},
//		{5, 6, 7, 8},
//		{9, 10, 11, 12},
//	}
//
// 1Byte 1KB = 1024Bytes 1MB = 1024KB = 1,048,576Bytes
// 定义8个方向（包括对角线）
var eightDirSlice = [][]int{
	{-1, 0}, // left 上 //index 0
	{0, 1},  // up 右  //index 1
	{1, 0},  // right 下  //index 2
	{0, -1}, // down 左 //index 3
	//
	{-1, 1},  //left up 右上 //index 4
	{1, 1},   //right up 右下 //index 5
	{1, -1},  //right down 左下  //index 6
	{-1, -1}, //left down 左上 //index 7
}

// 反向邻居坐标索引映射
var biDirNeighborMapping = []int{
	2, // left 上 //index 0
	3, // up 右  //index 1
	0, // right 下  //index 2
	1, // down 左 //index 3
	//
	6, //left up 右上 //index 4
	7, //right up 右下 //index 5
	4, //right down 左下  //index 6
	5, //left down 左上 //index 7
}

// 4方向分割的水平 垂直 索引
var eightDir_HV_Neighbor_Slice = [][]int{
	{0, 1}, //left up 右 上 //index 0
	{1, 2}, //right up 右 下 //index 1
	{2, 3}, //right down 左 下  //index 2
	{3, 0}, //left down 左 上 //index 3
}

const (
	Bit0   uint8 = 1 << 0     //00000001  上 (↑)
	Bit1   uint8 = 1 << 1     //00000010  右 (→)
	Bit2   uint8 = 1 << 2     //00000100	下 (↓)
	Bit3   uint8 = 1 << 3     //00001000	左 (←)
	Bit4   uint8 = 1 << 4     //00010000 右上 (↗)
	Bit5   uint8 = 1 << 5     //00100000 右下 (↘)
	Bit6   uint8 = 1 << 6     //01000000 左下 (↙)
	Bit7   uint8 = 1 << 7     //10000000  左上 (↖)
	BitALL uint8 = 0b11111111 //11111111 全方向
)

// Node
type Node struct {
	X      int     // 节点X坐标 4
	Y      int     // 节点Y坐标 4
	G      float32 // 从起点到当前节点的实际代价(考虑了障碍) 4
	F      float32 // F = G + H 4
	Parent *Node   // 父节点 8
	//IndexInOpenList
	IndexInOpenList int //当前节点在OpenList的索引 4
	//IsPOCWOC hasParent->00100000 OpenList2->00010000 CloseList2->00001000 Wall->00000100 OpenList1->00000010 CloseList1->00000001
	IsPOCWOC byte //1
	//DRA相关
	IIndex             int  //4 处于哪个索引分类的感兴趣墙角
	neighborIsMapNode  byte //00000001代表只有上邻居是地图内的点 --当然实际矩形地图不可能是这样
	neighborIsObstacle byte //00000001代表上邻居是障碍 --0代表不是障碍 1代表是障碍 顺时针方向 先上右下左 右上 右下 左下 左上
	//双向A相关(已废弃 后续考虑删除)
	ObstacleSAT int //4
	//要移动的8向存储 eight dir
	moveDir     byte //1
	singleIndex int  //每个节点唯一id -从0开始 从左上到右下
	// 测试标记:标志访问过
	isOut bool
}

// Map
type Map struct {
	Rows, Cols                  int
	startNode                   *Node
	endNode                     *Node
	ds                          float32 //当前起点终点的对角线距离
	Nodes                       [][]*Node
	Nodes_BitMap                [][]byte       //位图 0代表非障碍 1代表障碍
	nowObstacleNodeDic          map[*Node]int  //当前存储的障碍物dic v是障碍物数量
	nowObstacleNodeSlice        []*Node        //当前存储的障碍物切片
	dynamicNeedCheckObstacleDic map[*Node]bool //种子填充计算障碍物最大尺寸 k墙 v是否被访问过
	//A or impBTA or Mgr or DynamicRayCast
	openList1              []*Node
	openList2              []*Node
	closeList1             []*Node
	closeList2             []*Node
	hasParentList          []*Node
	OriginalNodeSlice      []*Node          //给Jps回溯open节点重现路径使用
	oldLabelForNodeDic     map[*Node]int    //k:obstacleNode v:label
	FinalLabelForNodeDic   map[int]*[]*Node //k:label v:nodeSlice
	obstacleNodeMaxSizeDic map[*Node]int    //k:obstacleNode v:obstacleMaxSize
	InitPath               []*Node          //初始路径
	//Mgr bit
	LeftToRightBitMap  [][]uint8 //行位图 比如索引0,2 对应第0行的第2位图(位图索引从0开始计算) --只在第0行
	UpToDownColsBitMap [][]uint8 //列位图 比如索引0,2 对应第0列的第2位图(位图索引从0开始计算) --只在第0列
	//HPA
	chunkNums, chunkWidth, chunkHeight int
	//DynamicRayCast
	ObstacleEdgeIndex         int           //-- 障碍或道路改变时 重置为0
	Real_ObstacleEdgeSlice    [][]*Node     //障碍物边界集合 -- 障碍或道路改变时 重置切片 重置当前切片中节点的 Is_AJI_AEC 标志为0
	Real_InterestNodeSlice    [][]*Node     //感兴趣墙角集合 -- 障碍或道路改变时 重置切片 重置当前切片中节点的 Is_AJI_AEC 标志为0
	testPrint_InterestNodeMap map[*Node]int //打印感兴趣墙角字典
	//
	temMemoryObstacle_SeedJudge_SingleIndex_Slice    []int  //id:1-判断记录在障碍边界判断中
	temMemoryObstacle_SeedJudge_SingleIndex_BitMap   []byte //id:1-判断记录在障碍边界判断中
	temMemoryRoad_InterestedJudge_SingleIndex_Slice  []int  //id:2-判断记录在非障碍感兴趣点判断中
	temMemoryRoad_InterestedJudge_SingleIndex_BitMap []byte //id:2-判断记录在非障碍感兴趣点判断中
	//
	obstacleEdgeSeedNode_Pop_Slice []*Node //存储弹出下一个临时障碍边界种子节点切片
	//
	temInterest_TypeIndex_Slice              []int  //id:3-临时感兴趣障碍类别索引 --同下
	temInterest_TypeIndex_CheckRepeatBitMap  []byte //id:3-临时判断重复障碍类别索引位图
	temInterest_SingleIndexNode_Slice        []int  //id:4-临时不重复障碍节点索引 --只有在判断完所有类别障碍节点之后才能知道完整信息 --依据这个重置位图,而不需要遍历完整个位图重置
	temInterest_SingleNode_CheckRepeatBitMap []byte //id:4-临时判断重复障碍节点本身位图
	//2点连线
	checkLineRepeat_Slice  []int  //id:5 -临时判断本次2点连线的不重复经过点id整数切片
	checkLineRepeat_BitMap []byte //id:5 -临时判断本次2点连线的不重复经过点id映射位图
	//后处理 	//step1 2
	midNodeSliceStep1 []*Node //step1需要使用的中间点
	//step2
	TemResultSliceStep2 []*Node       //第2步临时结果切片
	SMNodeSlice         []*Node       //step2的SM节点切片
	MENodeSlice         []*Node       //step2的ME节点切片
	NowNotRepeatNodeMap map[*Node]int //step2临时存储遍历经过的节点 不包括障碍
	//
	ResultNodeList []*Node
	//耗时相关
	PathFindDuration int64
	pushTimes        int //加入open的次数
	FixTimes         int //调整open的次数
	popTimes         int //弹出open的次数
}

type temIndex struct {
	endX int
	endY int
}

func (m *Map) ResetDynamicRayCastMap() {
	for i := 0; i < len(m.openList1); i++ {
		resetNode(m.openList1[i])
	}
	for i := 0; i < len(m.closeList1); i++ {
		resetNode(m.closeList1[i])
	}
	//
	m.startNode = nil
	m.endNode = nil
	m.ds = 0
	//
	m.openList1 = m.openList1[:0]
	m.closeList1 = m.closeList1[:0]
	m.InitPath = m.InitPath[:0]
	//step1
	m.midNodeSliceStep1 = m.midNodeSliceStep1[:0]
	//step2
	m.TemResultSliceStep2 = m.TemResultSliceStep2[:0] //第2步临时结果切片
	m.SMNodeSlice = m.SMNodeSlice[:0]                 //step2的SM节点切片
	m.MENodeSlice = m.MENodeSlice[:0]                 //step2的ME节点切片
	clear(m.NowNotRepeatNodeMap)                      //step2临时存储遍历经过的节点 不包括障碍
	//m.AfterDoublePointPath = m.AfterDoublePointPath[:0] // 重置后处理路径
	m.ResultNodeList = m.ResultNodeList[:0]
	//
	m.PathFindDuration = 0
	m.pushTimes = 0
	m.FixTimes = 0
	m.popTimes = 0
	//m.PathFindDuration = 0
}

func (m *Map) ResetAstarMap() {
	for i := 0; i < len(m.openList1); i++ {
		resetNode(m.openList1[i])
	}
	for i := 0; i < len(m.closeList1); i++ {
		resetNode(m.closeList1[i])
	}
	//
	m.startNode = nil
	m.endNode = nil
	m.ds = 0
	//
	m.openList1 = m.openList1[:0]
	m.closeList1 = m.closeList1[:0]
	m.InitPath = m.InitPath[:0]
	//step1
	m.midNodeSliceStep1 = m.midNodeSliceStep1[:0]
	//step2
	m.TemResultSliceStep2 = m.TemResultSliceStep2[:0] //第2步临时结果切片
	m.SMNodeSlice = m.SMNodeSlice[:0]                 //step2的SM节点切片
	m.MENodeSlice = m.MENodeSlice[:0]                 //step2的ME节点切片
	clear(m.NowNotRepeatNodeMap)                      //step2临时存储遍历经过的节点 不包括障碍
	//m.AfterDoublePointPath = m.AfterDoublePointPath[:0] // 重置后处理路径
	m.ResultNodeList = m.ResultNodeList[:0]
	//
	m.PathFindDuration = 0
	m.pushTimes = 0
	m.FixTimes = 0
	m.popTimes = 0
	//m.PathFindDuration = 0
}

func (m *Map) ResetJpsBitMap() {
	for i := 0; i < len(m.openList1); i++ {
		resetNode(m.openList1[i])
	}
	for i := 0; i < len(m.closeList1); i++ {
		resetNode(m.closeList1[i])
	}
	for i := 0; i < len(m.hasParentList); i++ {
		resetNode(m.hasParentList[i])
	}
	//
	m.startNode = nil
	m.endNode = nil
	m.ds = 0
	//
	m.openList1 = m.openList1[:0]
	m.closeList1 = m.closeList1[:0]
	m.hasParentList = m.hasParentList[:0]
	m.OriginalNodeSlice = m.OriginalNodeSlice[:0]
	m.InitPath = m.InitPath[:0]
	//step1
	m.midNodeSliceStep1 = m.midNodeSliceStep1[:0]
	//step2
	m.TemResultSliceStep2 = m.TemResultSliceStep2[:0] //第2步临时结果切片
	m.SMNodeSlice = m.SMNodeSlice[:0]                 //step2的SM节点切片
	m.MENodeSlice = m.MENodeSlice[:0]                 //step2的ME节点切片
	clear(m.NowNotRepeatNodeMap)                      //step2临时存储遍历经过的节点 不包括障碍
	//m.AfterDoublePointPath = m.AfterDoublePointPath[:0] // 重置后处理路径
	m.ResultNodeList = m.ResultNodeList[:0]
	//
	m.PathFindDuration = 0
	m.pushTimes = 0
	m.FixTimes = 0
	m.popTimes = 0
	//m.PathFindDuration = 0
}

func (m *Map) ResetIMPBThtAMap() {
	//openList1
	for i := 0; i < len(m.openList1); i++ {
		resetNode(m.openList1[i])
	}
	//closeList1
	for i := 0; i < len(m.closeList1); i++ {
		resetNode(m.closeList1[i])
	}
	//openList2
	for i := 0; i < len(m.openList2); i++ {
		resetNode(m.openList2[i])
	}
	//closeList2
	for i := 0; i < len(m.closeList2); i++ {
		resetNode(m.closeList2[i])
	}
	//
	m.startNode = nil
	m.endNode = nil
	m.ds = 0
	//
	m.openList1 = m.openList1[:0]
	m.closeList1 = m.closeList1[:0]
	//
	m.openList2 = m.openList2[:0]
	m.closeList2 = m.closeList2[:0]
	//
	m.InitPath = m.InitPath[:0]
	//step1
	m.midNodeSliceStep1 = m.midNodeSliceStep1[:0]
	//step2
	m.TemResultSliceStep2 = m.TemResultSliceStep2[:0] //第2步临时结果切片
	m.SMNodeSlice = m.SMNodeSlice[:0]                 //step2的SM节点切片
	m.MENodeSlice = m.MENodeSlice[:0]                 //step2的ME节点切片
	clear(m.NowNotRepeatNodeMap)                      //step2临时存储遍历经过的节点 不包括障碍
	//m.AfterDoublePointPath = m.AfterDoublePointPath[:0] // 重置后处理路径
	m.ResultNodeList = m.ResultNodeList[:0]
	//
	m.PathFindDuration = 0
	m.pushTimes = 0
	m.FixTimes = 0
	m.popTimes = 0
	//m.PathFindDuration = 0
}

var nodeCreateTimes int = -1

func NewNode(nodeX, nodeY int) *Node {
	nodeCreateTimes++
	return &Node{
		X:               nodeX,
		Y:               nodeY,
		G:               0,
		F:               0,
		Parent:          nil,
		IndexInOpenList: -1, //默认-1表示不再openList当中
		IsPOCWOC:        0,  //无标志
		ObstacleSAT:     0,  //积分默认为0
		moveDir:         0,  //无方向
		//
		singleIndex: nodeCreateTimes,
		//
		isOut: false,
	}
}

func NewDynamicRayCastNode(nodeX, nodeY int) *Node {
	nodeCreateTimes++
	return &Node{
		X:               nodeX,
		Y:               nodeY,
		G:               0,
		F:               0,
		Parent:          nil,
		IndexInOpenList: -1, //默认-1表示不再openList当中
		IsPOCWOC:        0,  //无标志
		IIndex:          -1, //默认不知道属于哪个障碍物类别
		//
		singleIndex: nodeCreateTimes,
		//
		isOut: false,
	}
}

func resetNode(node *Node) {
	//除了 XY 和 IsWall 其余都不动
	node.F = 0
	node.G = 0
	node.Parent = nil
	node.IndexInOpenList = -1
	//障碍物不动 其余归0
	node.clearOpen_Close_Parent_SignFlag()
	//重置为无move方向
	node.moveDir = 0
	//
	node.isOut = false
}

const Rows, Cols = 500, 500

// NewDynamicRayCastMap
func NewDynamicRayCastMap(rows, cols int) *Map {
	//Nodes
	//nodes := make([][]*Node, rows) //行
	//[[w0..],
	// [w1..],
	// [w2..],
	// [w3..],
	// [w4..]]
	//arr := [3r][4c]int{
	//	{1, 2, 3, 4},
	//	{5, 6, 7, 8},
	//	{9, 10, 11, 12},
	//}
	//arr[x:2][y:1] = 10
	nodeCreateTimes = -1
	var nodeSlice = make([][]*Node, rows)
	var nodeSlice_BitMap = make([][]byte, rows)
	obstacleEdgeSlice := make([][]*Node, 0, rows*cols/4)
	interestNodeSlice := make([][]*Node, 0, rows*cols/4)
	// 初始化二维切片的每一行
	for i := range nodeSlice {
		nodeSlice[i] = make([]*Node, cols)
	}
	for i := range nodeSlice_BitMap {
		nodeSlice_BitMap[i] = make([]byte, (cols/8)+1)
	}
	for i := 0; i < rows; i++ {
		for j := 0; j < cols; j++ {
			nodeSlice[i][j] = NewDynamicRayCastNode(i, j)
		}
	}
	for i := 0; i < rows*cols/4; i++ {
		temNodeSlice1, temNodeSlice2 := make([]*Node, 0, max(rows, cols)), make([]*Node, 0, max(rows, cols))
		obstacleEdgeSlice = append(obstacleEdgeSlice, temNodeSlice1)
		interestNodeSlice = append(interestNodeSlice, temNodeSlice2)
	}
	m := &Map{
		Rows:         rows, //行数
		Cols:         cols, //列数
		Nodes:        nodeSlice,
		Nodes_BitMap: nodeSlice_BitMap,
		// 预分配
		openList1:        make([]*Node, 0, 1024*32),
		closeList1:       make([]*Node, 0, 1024*16),
		PathFindDuration: 0,
		InitPath:         make([]*Node, 0, 128),
		//
		ObstacleEdgeIndex:                                0,                             //-- 障碍或道路改变时 重置为0
		Real_ObstacleEdgeSlice:                           obstacleEdgeSlice,             //障碍物边界集合 -- 障碍或道路改变时 重置切片 重置当前切片中节点的 Is_AJI_AEC 标志为0
		Real_InterestNodeSlice:                           interestNodeSlice,             //感兴趣墙角集合 -- 障碍或道路改变时 重置切片 重置当前切片中节点的 Is_AJI_AEC 标志为0
		testPrint_InterestNodeMap:                        make(map[*Node]int),           //测试用 记得注释！！！！！！！！！
		temMemoryObstacle_SeedJudge_SingleIndex_Slice:    make([]int, 0, 1024),          //判断记录在障碍边界判断中
		temMemoryObstacle_SeedJudge_SingleIndex_BitMap:   make([]byte, (rows*cols/8)+1), //在障碍边界或非障碍感兴趣节点判断中 不是目标点的判断记忆记录 -- 障碍或道路改变时
		temMemoryRoad_InterestedJudge_SingleIndex_Slice:  make([]int, 0, 1024),          //判断记录在非障碍感兴趣点判断中
		temMemoryRoad_InterestedJudge_SingleIndex_BitMap: make([]byte, (rows*cols/8)+1), //判断记录在非障碍感兴趣点判断中
		//
		obstacleEdgeSeedNode_Pop_Slice:           make([]*Node, 0, 1024),        //计算临时障碍边界种子节点切片
		temInterest_TypeIndex_Slice:              make([]int, 0, 1024),          //临时感兴趣障碍类别索引
		temInterest_TypeIndex_CheckRepeatBitMap:  make([]byte, (rows*cols/8)+1), //临时判断重复障碍类别索引位图
		temInterest_SingleIndexNode_Slice:        make([]int, 0, 1024),          //临时不重复障碍节点索引 --只有在判断完所有类别障碍节点之后才能知道完整信息 --依据这个重置位图,而不需要遍历完整个位图重置
		temInterest_SingleNode_CheckRepeatBitMap: make([]byte, (rows*cols/8)+1), //临时判断重复障碍节点本身位图
		//2点连线相关
		checkLineRepeat_Slice:  make([]int, 0, 1024),          //临时判断2点连线去重经过点
		checkLineRepeat_BitMap: make([]byte, (rows*cols/8)+1), //临时判断2点连线去重经过点位图,位图容量要+1，因为差1位，比如10*10,需要13个byte,不加1则100/8 = 12,实际只能检测96个id所以要+1
		//step1
		midNodeSliceStep1: make([]*Node, 0, 64),
		//step2
		TemResultSliceStep2: make([]*Node, 0, 64),
		SMNodeSlice:         make([]*Node, 0, 1024*4),
		MENodeSlice:         make([]*Node, 0, 1024*4),
		NowNotRepeatNodeMap: make(map[*Node]int, 1024*2),
		//ResultNodeList
		ResultNodeList: make([]*Node, 0, 16),
		//
		pushTimes: 0,
		popTimes:  0,
	}
	//再次初始化邻居越界和障碍信息
	for i := 0; i < rows; i++ {
		for j := 0; j < cols; j++ {
			m.setDynamicRayCast_NodeNeighbor_Info(m.Nodes[i][j])
		}
	}
	//预计算通用2点连线坐标点
	//m.calLineObstacle_PreLoad()
	return m
}

// NewAstarMap
func NewAstarMap(rows, cols int) *Map {
	//Nodes
	//nodes := make([][]*Node, rows) //行
	//[[w0..],
	// [w1..],
	// [w2..],
	// [w3..],
	// [w4..]]
	//arr := [3r][4c]int{
	//	{1, 2, 3, 4},
	//	{5, 6, 7, 8},
	//	{9, 10, 11, 12},
	//}
	//arr[x:2][y:1] = 10
	nodeCreateTimes = -1
	var twoDSlice = make([][]*Node, rows)
	// 初始化二维切片的每一行
	for i := range twoDSlice {
		twoDSlice[i] = make([]*Node, cols)
	}
	for i := 0; i < rows; i++ {
		for j := 0; j < cols; j++ {
			twoDSlice[i][j] = NewNode(i, j)
		}
	}
	m := &Map{
		Rows:  rows, //行数
		Cols:  cols, //列数
		Nodes: twoDSlice,
		// 预分配
		openList1:        make([]*Node, 0, 1024*32),
		closeList1:       make([]*Node, 0, 1024*16),
		PathFindDuration: 0,
		InitPath:         make([]*Node, 0, 128),
		//step1
		midNodeSliceStep1: make([]*Node, 0, 64),
		//step2
		TemResultSliceStep2: make([]*Node, 0, 64),
		SMNodeSlice:         make([]*Node, 0, 1024*4),
		MENodeSlice:         make([]*Node, 0, 1024*4),
		NowNotRepeatNodeMap: make(map[*Node]int, 1024*2),
		//ResultNodeList
		ResultNodeList: make([]*Node, 0, 16),
		//
		pushTimes: 0,
		popTimes:  0,
	}
	return m
}

// NewImpBiThetAstarMap
func NewImpBiThetAstarMap(rows, cols int) *Map {
	nodeCreateTimes = -1
	var twoDSlice = make([][]*Node, rows)
	// 初始化二维切片的每一行
	for i := range twoDSlice {
		twoDSlice[i] = make([]*Node, cols)
	}
	for i := 0; i < rows; i++ {
		for j := 0; j < cols; j++ {
			twoDSlice[i][j] = NewNode(i, j)
		}
	}
	m := &Map{
		Rows:  rows, //行数
		Cols:  cols, //列数
		Nodes: twoDSlice,
		//impBTA计算障碍物最大尺寸用到
		nowObstacleNodeDic:          make(map[*Node]int, (rows*cols)/8),  //当前障碍物dic 理解为无地图顺序
		nowObstacleNodeSlice:        make([]*Node, 0, (rows*cols)/8),     //当前障碍物Slice 理解为无地图顺序 注意长度0和容量
		dynamicNeedCheckObstacleDic: make(map[*Node]bool, (rows*cols)/8), //当前动态障碍物dic k v是否访问过
		// 预分配
		openList1:  make([]*Node, 0, 1024*32),
		closeList1: make([]*Node, 0, 1024*16),
		openList2:  make([]*Node, 0, 1024*32),
		closeList2: make([]*Node, 0, 1024*16),
		// obstacle max size,same len
		oldLabelForNodeDic:     make(map[*Node]int, 1024*16),   //这个可以计算完缩容
		FinalLabelForNodeDic:   make(map[int]*[]*Node, 1024*8), //这个可以计算完缩容
		obstacleNodeMaxSizeDic: make(map[*Node]int, 1024*16),   //这个计算好了就不动了
		//
		PathFindDuration: 0,
		InitPath:         make([]*Node, 0, 128),
		//step1
		midNodeSliceStep1: make([]*Node, 0, 64),
		//step2
		TemResultSliceStep2: make([]*Node, 0, 64),
		SMNodeSlice:         make([]*Node, 0, 1024*4),
		MENodeSlice:         make([]*Node, 0, 1024*4),
		NowNotRepeatNodeMap: make(map[*Node]int, 1024*2),
		//ResultNodeList
		ResultNodeList: make([]*Node, 0, 16),
		//
		pushTimes: 0,
		popTimes:  0,
	}
	return m
}

// NewJpsBitMap 位运算bit版本创建地图
func NewJpsBitMap(rows, cols int) *Map {
	//Nodes
	//nodes := make([][]*Node, rows) //行
	//[[w0..],
	// [w1..],
	// [w2..],
	// [w3..],
	// [w4..]]
	//arr := [3r][4c]int{
	//	{1, 2, 3, 4},
	//	{5, 6, 7, 8},
	//	{9, 10, 11, 12},
	//}
	//arr[x:2][y:1] = 10
	nodeCreateTimes = -1
	var twoDSlice = make([][]*Node, rows)
	leftToRightRowsBitMap := make([][]uint8, rows) //比如索引0,2 对应第0行的第2位图(位图索引从0开始计算)
	upToDownColsBitMap := make([][]uint8, cols)    //比如索引0,2 对应第0列的第2位图(位图索引从0开始计算)
	// 初始化二维切片的每一行
	for i := range twoDSlice {
		twoDSlice[i] = make([]*Node, cols)
		leftToRightRowsBitMap[i] = make([]uint8, 0, (cols/8)+1) //长度和容量是列的数量除8的整数位 + 1
		upToDownColsBitMap[i] = make([]uint8, 0, (rows/8)+1)    //长度和容量是行的数量除8的整数位 + 1
	}
	for r := 0; r < rows; r++ {
		leftToRightRowsBitMap[r] = make([]uint8, 0, (cols/8)+1) //长度和容量是列的数量除8的整数位 + 1
	}
	for c := 0; c < cols; c++ {
		upToDownColsBitMap[c] = make([]uint8, 0, (rows/8)+1) //长度和容量是行的数量除8的整数位 + 1
	}
	for i := 0; i < rows; i++ {
		for j := 0; j < cols; j++ {
			twoDSlice[i][j] = NewNode(i, j)
		}
	}
	m := &Map{
		Rows:  rows, //行数
		Cols:  cols, //列数
		Nodes: twoDSlice,
		// 预分配
		openList1:         make([]*Node, 0, 1024*32),
		closeList1:        make([]*Node, 0, 1024*16),
		hasParentList:     make([]*Node, 0, 1024*8),
		OriginalNodeSlice: make([]*Node, 0, 3),
		PathFindDuration:  0,
		InitPath:          make([]*Node, 0, 128),
		//额外:位图
		LeftToRightBitMap:  leftToRightRowsBitMap,
		UpToDownColsBitMap: upToDownColsBitMap,
		//step1
		midNodeSliceStep1: make([]*Node, 0, 64),
		//step2
		TemResultSliceStep2: make([]*Node, 0, 64),
		SMNodeSlice:         make([]*Node, 0, 1024*4),
		MENodeSlice:         make([]*Node, 0, 1024*4),
		NowNotRepeatNodeMap: make(map[*Node]int, 1024*2),
		//ResultNodeList
		ResultNodeList: make([]*Node, 0, 16),
		//
		pushTimes: 0,
		popTimes:  0,
	}
	return m
}

// -------------------------------------------------------计算Jps位运算位图相关

// //2.从后往前更新 反向行位图
// for i := 0; i < m.Rows; i++ {
// //新增 1行的最后一个位图索引开始更新
// reverseSaveTimes = len(m.RightToLeftBitMap[i]) - 1
// for j := m.Cols - 1; j >= 0; j-- {
// //不断或预算新的对应障碍位置二进制
// if m.Nodes[i][j].IsWall() {
// saveBit |= 1 << times //注意1的二进制不断左移
// }
// //当超过次数7之后 代表进入到下一个位图 存储这次位图 然后重新刷新bitDefault
// if times == 7 {
// m.RightToLeftBitMap[i][reverseSaveTimes] = saveBit
// //记得重置 saveBit  times
// saveBit, times = 0, 0
// reverseSaveTimes--
// continue //注意continue
// }
// //如果当前是最后一位索引 且 times != 7 新建1个剩余位全是障碍的二进制 进行或运算 然后直接存储结束本行
// if j == 0 {
// //循环1左移
// for k := 0; k < 7-times; k++ {
// lastBit |= 128 >> k
// }
// //然后或运算 存储
// saveBit |= lastBit
// m.RightToLeftBitMap[i][reverseSaveTimes] = saveBit
// reverseSaveTimes--
// //记得重置 saveBit  times
// saveBit, times = 0, 0
// continue //注意continue
// }
// times++
// }
// }
// CalJpsBitMap_AfterSetWall 注意该方法只会在初始化地图调用
func (m *Map) CalJpsBitMap_AfterSetWall() {
	//start:先定义一个
	//0.拟定一个二进制0 偏移位置就是遍历次数,当遇到障碍物的时候,置当前位为1
	var saveBit uint8 = 0
	var lastBit uint8 = 0
	var times = 0 //每7位重置一次 如果碰到边界发现没填满 那么用一个1左移没填满次数的二进制或运算补齐后面所有1
	//-------------------------------------------更新所有行位图(8位)
	//1.逐个遍历行 每8位障碍信息存储一次 正向行位图
	for i := 0; i < m.Rows; i++ {
		for j := 0; j < m.Cols; j++ {
			//不断或预算新的对应障碍位置二进制
			if m.Nodes[i][j].IsWall() {
				saveBit |= 128 >> times //注意是128的二进制不断右移
			}
			//当超过次数7之后 代表进入到下一个位图 存储这次位图 然后重新刷新bitDefault
			if times == 7 {
				m.LeftToRightBitMap[i] = append(m.LeftToRightBitMap[i], saveBit) //当前啊行逐个添加
				//记得重置 saveBit  times
				saveBit, times = 0, 0
				continue //注意continue
			}
			//如果当前是最后一位索引 且 times != 7 新建1个剩余位全是障碍的二进制 进行或运算 然后直接存储结束本行
			if j == m.Cols-1 {
				//循环1左移
				for k := 0; k < 7-times; k++ {
					lastBit |= 1 << k
				}
				//然后或运算 存储
				saveBit |= lastBit
				m.LeftToRightBitMap[i] = append(m.LeftToRightBitMap[i], saveBit)
				//记得重置 saveBit  times
				saveBit, times = 0, 0
				continue //注意continue
			}
			times++
		}
	}
	//重置
	saveBit, lastBit, times = 0, 0, 0
	//-------------------------------------------更新所有列位图(8位)
	//正向列位图
	for j := 0; j < m.Cols; j++ {
		for i := 0; i < m.Rows; i++ {
			//不断或运算新的对应障碍位置二进制
			if m.Nodes[i][j].IsWall() {
				saveBit |= 128 >> times //注意是128的二进制不断右移
			}
			//当超过次数7之后 代表进入到下一个位图 存储这次位图 然后重新刷新bitDefault
			if times == 7 {
				m.UpToDownColsBitMap[j] = append(m.UpToDownColsBitMap[j], saveBit) //当前列逐个添加
				//记得重置 saveBit  times
				saveBit, times = 0, 0
				continue //注意continue
			}
			//如果当前是最后一位索引 且 times != 7 新建1个剩余位全是障碍的二进制 进行或运算 然后直接存储结束本行
			if i == m.Rows-1 {
				//循环1左移
				for k := 0; k < 7-times; k++ {
					lastBit |= 1 << k
				}
				//然后或运算 存储
				saveBit |= lastBit
				m.UpToDownColsBitMap[j] = append(m.UpToDownColsBitMap[j], saveBit) //当前列逐个添加
				//记得重置 saveBit  times
				saveBit, times = 0, 0
				continue //注意continue
			}
			times++
		}
	}
	//-------------------------------------------------查看位图数据
	fmt.Println("行位图-正向：")
	for i := 0; i < len(m.LeftToRightBitMap); i++ {
		for j := 0; j < len(m.LeftToRightBitMap[i]); j++ {
			//打印每行二进制信息
			fmt.Printf("%08b", m.LeftToRightBitMap[i][j])
		}
		fmt.Println()
	}
	//fmt.Printf("%08b", m.UpToDownColsBitMap[0][1])
	//fmt.Printf("%08b", m.UpToDownColsBitMap[0][2])
	//fmt.Printf("%08b", m.UpToDownColsBitMap[0][3])
	//fmt.Printf("%08b", m.UpToDownColsBitMap[0][4])
	fmt.Println("列位图--正向：")
	for i := 0; i < len(m.UpToDownColsBitMap); i++ {
		for j := 0; j < len(m.UpToDownColsBitMap[i]); j++ {
			//fmt.Println(i, " ", j)
			//打印每行二进制信息
			fmt.Printf("%08b", m.UpToDownColsBitMap[i][j])
		}
		fmt.Println()
	}
}

// -------------------------------------------------------IMBTA计算障碍物分布相关
func (m *Map) CalObstacleSAT() {
	// 0 1 0 0
	// 1 1 0 0
	// 1 0 1 0
	//1.记录当前行累计的障碍数量 下一行重置为0
	nowLineIncreaseObstacle := 0
	//2.必须要求地图为2行及以上 先计算好第1行的node与原点围成的矩形区域的总障碍数量
	if m.Rows < 2 {
		panic("地图行数低于2行 创建地图失败")
	} else {
		for l := 0; l < m.Cols; l++ {
			if m.Nodes[0][l].IsWall() {
				nowLineIncreaseObstacle++
			}
			//save node Obstacle SAT
			m.Nodes[0][l].ObstacleSAT = nowLineIncreaseObstacle
		}
	}
	//3.从第2行开始逐行遍历地图 设置每个node与原点围成的矩形区域的总障碍数量
	for i := 1; i < m.Rows; i++ {
		//4.注意重置
		nowLineIncreaseObstacle = 0
		for j := 0; j < m.Cols; j++ {
			//累计当前行的障碍物数量
			if m.Nodes[i][j].IsWall() {
				nowLineIncreaseObstacle++
			}
			//save SAT = nowLineIncreaseObstacle + 上一行当前列存储的SAT
			m.Nodes[i][j].ObstacleSAT = nowLineIncreaseObstacle + m.Nodes[i-1][j].ObstacleSAT
		}
	}
	fmt.Println("----------SAT--------------")
	//4.打印当前SAT
	for i := 0; i < m.Rows; i++ {
		for j := 0; j < m.Cols; j++ {
			fmt.Print(m.Nodes[i][j].ObstacleSAT)
		}
		fmt.Println()
	}
}

func (m *Map) CalObstacleMax_4Dir() {
	//Two loop
	//1.first loop 从左上遍历到右下 障碍node本身检查规则：1.如果障碍node左和上越界或者都不是障碍 该障碍node记录标签 标签+1
	label := 1 //标签从1开始
	leftIsMapNode := false
	upIsMapNode := false
	var nowNode, leftNode, downNode *Node
	for i := 0; i < m.Rows; i++ {
		for j := 0; j < m.Cols; j++ {
			//1.如果障碍node左和上越界或者都不是障碍 该障碍node记录标签 标签+1
			if m.Nodes[i][j].IsWall() {
				//1.当前node
				nowNode = m.Nodes[i][j]
				//2.左节点 上节点
				leftIsMapNode = m.isMapNode(i, j-1)
				upIsMapNode = m.isMapNode(i-1, j)
				if leftIsMapNode {
					leftNode = m.Nodes[i][j-1]
				}
				if upIsMapNode {
					downNode = m.Nodes[i-1][j]
				}
				//3.左上都越界
				if !leftIsMapNode && !upIsMapNode {
					m.oldLabelForNodeDic[nowNode] = label
					label++
					continue
				} else if !leftIsMapNode { //左越界 上不越界
					//4.上是墙 继承上
					if downNode.IsWall() {
						m.oldLabelForNodeDic[nowNode] = m.oldLabelForNodeDic[downNode]
					} else { //上不是墙 新label
						m.oldLabelForNodeDic[nowNode] = label
						label++
					}
					continue
				} else if !upIsMapNode { //左不越界 上越界
					//5.左是墙 继承左
					if leftNode.IsWall() {
						m.oldLabelForNodeDic[nowNode] = m.oldLabelForNodeDic[leftNode]
					} else { //左不是墙 新label
						m.oldLabelForNodeDic[nowNode] = label
						label++
					}
					continue
				} else { //左上都不越界
					//6.判断左上可能都是障碍 或者左是障碍 或者上是障碍
					if leftNode.IsWall() && downNode.IsWall() { //都是墙 继承小的label
						m.oldLabelForNodeDic[nowNode] = getSmallerInt(m.oldLabelForNodeDic[leftNode], m.oldLabelForNodeDic[downNode])
					} else if downNode.IsWall() { //谁是墙 就是继承这个label
						m.oldLabelForNodeDic[nowNode] = m.oldLabelForNodeDic[downNode]
					} else if leftNode.IsWall() { //谁是墙 就是继承这个label
						m.oldLabelForNodeDic[nowNode] = m.oldLabelForNodeDic[leftNode]
					} else { //都不是墙 新label
						m.oldLabelForNodeDic[nowNode] = label
						label++
					}
					continue
				}
			}
		}
	}

	fmt.Println("----------------first save old ----------")
	//2.打印看看 并且为当前label创建第1次遍历后的切片存储(有映射切片则添加 无映射则创建切片映射)
	for i := 0; i < m.Rows; i++ {
		for j := 0; j < m.Cols; j++ {
			nowNode = m.Nodes[i][j]
			//只有障碍物才有label 其余不存储 这里用0表示 实际没有存储只是为了打印区分
			if oldLabel, ok := m.oldLabelForNodeDic[nowNode]; ok {
				if nodeSlice, ok1 := m.FinalLabelForNodeDic[oldLabel]; ok1 {
					*nodeSlice = append(*nodeSlice, nowNode)
				} else {
					newNodeSlice := make([]*Node, 0, 0)
					m.FinalLabelForNodeDic[oldLabel] = &newNodeSlice
					newNodeSlice = append(newNodeSlice, nowNode)
				}
				fmt.Print(oldLabel)
			} else {
				fmt.Print(0)
			}
		}
		fmt.Println()
	}
	//3.左上到右下遍历墙 如果墙的label大于右边的墙label 那么所有相同的大的label都要设置为这个小的label(这里做的操作是把当前大label切片添加进小的切片中 然后删除当前大label切片映射)
	var rightNode *Node
	rightNodeIsInMap := false
	biggerLabel := 0
	for i := 0; i < m.Rows; i++ {
		for j := 0; j < m.Cols; j++ {
			//当前node 右node
			nowNode = m.Nodes[i][j]
			rightNodeIsInMap = m.isMapNode(i, j+1)
			if rightNodeIsInMap {
				rightNode = m.Nodes[i][j+1]
			}
			//4.当前是墙 右界内且是墙
			if nowNode.IsWall() && rightNodeIsInMap && rightNode.IsWall() {
				//5.当前label大于右label 说明之前是第2象限的连接形状 把当前大label切片转移添加进小的切片中 然后删除当前大label切片映射
				if m.oldLabelForNodeDic[nowNode] > m.oldLabelForNodeDic[rightNode] {
					biggerLabel = m.oldLabelForNodeDic[nowNode]
					//转移大 -> 小
					*m.FinalLabelForNodeDic[m.oldLabelForNodeDic[rightNode]] = append(*m.FinalLabelForNodeDic[m.oldLabelForNodeDic[rightNode]],
						*m.FinalLabelForNodeDic[m.oldLabelForNodeDic[nowNode]]...)
					//注意！更新所有小切片新值到旧里面
					for k := 0; k < len(*m.FinalLabelForNodeDic[m.oldLabelForNodeDic[rightNode]]); k++ {
						m.oldLabelForNodeDic[(*m.FinalLabelForNodeDic[m.oldLabelForNodeDic[rightNode]])[k]] = m.oldLabelForNodeDic[rightNode]
					}
					//删除大
					delete(m.FinalLabelForNodeDic, biggerLabel)
				}
			}
		}
	}

	//4.最后用最终的更新旧
	for finalLabel, finalNodeSlice := range m.FinalLabelForNodeDic {
		for i := 0; i < len(*finalNodeSlice); i++ {
			//每一个node的值都更新到旧的
			m.oldLabelForNodeDic[(*finalNodeSlice)[i]] = finalLabel
		}
	}

	fmt.Println("----------------check update old----------")
	//5.然后打印看看对不对
	for i := 0; i < m.Rows; i++ {
		for j := 0; j < m.Cols; j++ {
			nowNode = m.Nodes[i][j]
			//只有障碍物才有label 其余不存储 这里用0表示 实际没有存储只是为了打印区分
			if finalLabel, ok := m.oldLabelForNodeDic[nowNode]; ok {
				fmt.Print(finalLabel)
			} else {
				fmt.Print(0)
			}
		}
		fmt.Println()
	}

	fmt.Println("----------------Cal Obstacle Max Size And Save----------")
	//end:遍历finalDic 找一个映射切片中的左上点和右下点 然后看谁的哪个轴从差值大 最大的就是当前障碍物最大尺寸
	smallerX, biggerX, smallerY, biggerY := 0, 0, 0, 0
	maxObstacleSize := 0
	for _, finalSlice := range m.FinalLabelForNodeDic {
		//假设第0个是左上点 或者 右下点
		smallerX = (*finalSlice)[0].X
		smallerY = (*finalSlice)[0].Y
		biggerX = (*finalSlice)[0].X
		biggerY = (*finalSlice)[0].Y
		for i := 1; i < len(*finalSlice); i++ {
			//不断与后面的比较然后更新左上点和右下点
			smallerX = getSmallerInt(smallerX, (*finalSlice)[i].X)
			smallerY = getSmallerInt(smallerY, (*finalSlice)[i].Y)
			biggerX = getBiggerInt(biggerX, (*finalSlice)[i].X)
			biggerY = getBiggerInt(biggerY, (*finalSlice)[i].Y)
		}
		//最后 比较X和Y各自的差值 看谁的差值大 这个就是障碍物最大尺寸
		maxObstacleSize = getBiggerInt(biggerX-smallerX, biggerY-smallerY)
		//Save max obstacle size
		for i := 0; i < len(*finalSlice); i++ {
			m.obstacleNodeMaxSizeDic[(*finalSlice)[i]] = maxObstacleSize + 1 //注意这里是+1 障碍物比如3索引到0，实际长度尺寸是4
		}
	}

	fmt.Println("----------------check final max obstacle size----------")
	for i := 0; i < m.Rows; i++ {
		for j := 0; j < m.Cols; j++ {
			nowNode = m.Nodes[i][j]
			if saveMaxObstacleSize, ok := m.obstacleNodeMaxSizeDic[nowNode]; ok {
				fmt.Print(saveMaxObstacleSize)
			} else {
				fmt.Print(0)
			}
		}
		fmt.Println()
	}
}

func (m *Map) CalObstacleMax_8dir() {
	//种子填充法，当前只对墙进行8向检索--不是墙 或者 已经检索过的墙 都跳过
	//先清空动态检查dic
	clear(m.dynamicNeedCheckObstacleDic)
	//1.先复制原墙Slice1份到动态检索墙dic中 默认所有墙没有检索过 v：false
	for i := 0; i < len(m.nowObstacleNodeSlice); i++ {
		m.dynamicNeedCheckObstacleDic[m.nowObstacleNodeSlice[i]] = false
	}
	var nowFatherObstacleNode, nowNeighborObstacleNode *Node
	finalLabel := 0
	checkSlice := make([]*Node, 0, len(m.nowObstacleNodeSlice)/4)
	//2.还是遍历原墙slice 对每个墙进行8向检查
	for i := 0; i < len(m.nowObstacleNodeSlice); i++ {
		//0.当前父墙
		nowFatherObstacleNode = m.nowObstacleNodeSlice[i]
		//1.先判断当前根是否已经检索过 检索过则跳过
		if m.dynamicNeedCheckObstacleDic[nowFatherObstacleNode] {
			continue
		}
		//2.添加进拓展切片
		checkSlice = append(checkSlice, nowFatherObstacleNode)
		//3.父节点Loop
		for len(checkSlice) > 0 {
			//4.取出末尾当做新的父 直接设置为当前label 因为必定是墙 长度-1
			nowFatherObstacleNode = checkSlice[len(checkSlice)-1]
			checkSlice = checkSlice[:len(checkSlice)-1]
			//5.根墙当做新种子：如果当前8向邻居不是墙 或已经拓展过 则跳过
			for d := 0; d < len(eightDirSlice); d++ {
				//6.邻居有可能越界
				if m.isMapNode(nowFatherObstacleNode.X+eightDirSlice[d][0], nowFatherObstacleNode.Y+eightDirSlice[d][1]) {
					//6.邻居 eightDirSlice的元素是切片，切片索引0是X增量，索引1是Y增量
					nowNeighborObstacleNode = m.Nodes[nowFatherObstacleNode.X+eightDirSlice[d][0]][nowFatherObstacleNode.Y+eightDirSlice[d][1]]
				} else {
					continue
				}
				//7.邻居不知道是墙还是路 如果不是墙 或已经拓展过 则跳过
				if !nowNeighborObstacleNode.IsWall() || m.dynamicNeedCheckObstacleDic[nowNeighborObstacleNode] {
					continue
				}
				//8.否则添加进待拓展
				checkSlice = append(checkSlice, nowNeighborObstacleNode)
			}
			//Loop end:标记当前父已经拓展过了 不会再被添加
			m.dynamicNeedCheckObstacleDic[nowFatherObstacleNode] = true
			//当前父节点记录对应标签 -每次只对父节点记录标签
			m.appendInFinalLabel(finalLabel, nowFatherObstacleNode)
		}
		//Root end:对每个根进行label区分++ (因为每进行1个根检索 必定能找齐这次label的所有连通)
		finalLabel++
	}

	//end:当前FinalLabelForNodeDic已经是正确的障碍物分块标记 计算最大尺寸
	fmt.Println("----------------Cal Obstacle Max Size And Save----------")
	//end:遍历finalDic 找一个映射切片中的左上点和右下点 然后看谁的哪个轴从差值大 最大的就是当前障碍物最大尺寸
	smallerX, biggerX, smallerY, biggerY := 0, 0, 0, 0
	maxObstacleSize := 0
	for _, finalSlice := range m.FinalLabelForNodeDic {
		//假设第0个是左上点 或者 右下点
		smallerX = (*finalSlice)[0].X
		smallerY = (*finalSlice)[0].Y
		biggerX = (*finalSlice)[0].X
		biggerY = (*finalSlice)[0].Y
		for i := 1; i < len(*finalSlice); i++ {
			//不断与后面的比较然后更新左上点和右下点
			smallerX = getSmallerInt(smallerX, (*finalSlice)[i].X)
			smallerY = getSmallerInt(smallerY, (*finalSlice)[i].Y)
			biggerX = getBiggerInt(biggerX, (*finalSlice)[i].X)
			biggerY = getBiggerInt(biggerY, (*finalSlice)[i].Y)
		}
		//最后 比较X和Y各自的差值 看谁的差值大 这个就是障碍物最大尺寸
		maxObstacleSize = getBiggerInt(biggerX-smallerX, biggerY-smallerY)
		//Save max obstacle size
		for i := 0; i < len(*finalSlice); i++ {
			m.obstacleNodeMaxSizeDic[(*finalSlice)[i]] = maxObstacleSize + 1 //注意这里是+1 障碍物比如3索引到0，实际长度尺寸是4
		}
	}
	fmt.Println("----------------check final max obstacle size----------")
	var nowNode *Node
	for i := 0; i < m.Rows; i++ {
		for j := 0; j < m.Cols; j++ {
			nowNode = m.Nodes[i][j]
			if saveMaxObstacleSize, ok := m.obstacleNodeMaxSizeDic[nowNode]; ok {
				fmt.Print(saveMaxObstacleSize)
			} else {
				fmt.Print(0)
			}
		}
		fmt.Println()
	}
}

func (m *Map) appendInFinalLabel(label int, node *Node) {
	if slice, ok := m.FinalLabelForNodeDic[label]; ok {
		//已有映射 添加到切片尾部
		*slice = append(*slice, node)
	} else {
		//创建新切片添加 并添加映射
		newSlice := make([]*Node, 0, 1)
		newSlice = append(newSlice, node)
		m.FinalLabelForNodeDic[label] = &newSlice
	}
}

// -------------------------------------------------------DynamicRayCast清除记忆相关

// 只有真正新增加或减少了不同格子的障碍非障碍 才值得重新清除边界和墙角记忆
func (m *Map) clearNodeMemoryAfterWallExchange() {
	m.ObstacleEdgeIndex = 0
	//注意 是遍历里面的切片 然后重置切片 不是重置二维指针长度
	for i := 0; i < len(m.Real_ObstacleEdgeSlice); i++ {
		m.Real_ObstacleEdgeSlice[i] = m.Real_ObstacleEdgeSlice[i][:0]
		m.Real_InterestNodeSlice[i] = m.Real_InterestNodeSlice[i][:0]
	}
}

//---------------------------------------Map manager

// SetWall 设置某个节点为墙
func (m *Map) SetWall(x, y int) {
	if m.isMapNode(x, y) && !m.Nodes[x][y].IsWall() {
		m.Nodes[x][y].IsPOCWOC = Bit2
	}
}

// SetRoad 设置道路
func (m *Map) SetRoad(x, y int) {
	if m.isMapNode(x, y) && m.Nodes[x][y].IsWall() {
		m.Nodes[x][y].IsPOCWOC = 0
	}
}

// SetWall_DynamicRayCast 设置墙1
func (m *Map) SetWall_DynamicRayCast(x, y int) {
	if m.isMapNode(x, y) && !m.Nodes[x][y].IsWall() { //之前不是wall才能设置为wall 否则无意义
		m.Nodes[x][y].IsPOCWOC = Bit2
		m.Nodes_BitMap[x][y/8] |= 128 >> (y % 8) //修改位图信息：注意这里一定要加括号！
		m.clearNodeMemoryAfterWallExchange()     //暂时用清除所有记忆的做法
		m.setDynamicRayCast_NodeNeighbor_Info(m.Nodes[x][y])
	}
}

// SetRoad_DynamicRayCast 设置路0
func (m *Map) SetRoad_DynamicRayCast(x, y int) {
	if m.isMapNode(x, y) && m.Nodes[x][y].IsWall() { //之前不是road才能设置为road 否则无意义
		m.Nodes[x][y].IsPOCWOC = 0
		m.Nodes_BitMap[x][y/8] &= ^(128 >> (y % 8)) //修改位图信息：注意这里一定要加括号！
		m.clearNodeMemoryAfterWallExchange()        //暂时用清除所有记忆的做法
		m.setDynamicRayCast_NodeNeighbor_Info(m.Nodes[x][y])
	}
}

// 设置节点周围8邻居是否为障碍物的位图信息 参数node肯定是地图内的点
func (m *Map) setDynamicRayCast_NodeNeighbor_Info(node *Node) {
	//遍历该节点8邻居
	var neighbor *Node
	var neighborX, neighborY int
	var nodeIsWall = node.IsWall()
	for i := 0; i < len(eightDirSlice); i++ {
		neighborX, neighborY = eightDirSlice[i][0]+node.X, eightDirSlice[i][1]+node.Y
		if !m.isMapNode(neighborX, neighborY) {
			node.neighborIsMapNode &= ^(1 << i)
			continue
		}
		//邻居
		neighbor = m.Nodes[neighborX][neighborY]
		//节点邻居是地图内点
		node.neighborIsMapNode |= 1 << i
		neighbor.neighborIsMapNode |= 1 << biDirNeighborMapping[i]
		//判断中点是wall还是road road不需要处理
		if neighbor.IsWall() {
			//中点存储正向邻居信息
			node.neighborIsObstacle |= 1 << i
		} else {
			node.neighborIsObstacle &= ^(1 << i)
		}
		//邻居反向存储中点信息
		if nodeIsWall {
			neighbor.neighborIsObstacle |= 1 << biDirNeighborMapping[i]
		} else {
			neighbor.neighborIsObstacle &= ^(1 << biDirNeighborMapping[i])
		}
	}
}

// 查看节点邻居是否是墙 与运算
func (node *Node) checkNeighbor_IsMapNode(neighborIndex int) bool {
	return ((1 << neighborIndex) & node.neighborIsMapNode) != 0
}

// 查看节点邻居是否是墙 与运算
func (node *Node) checkNeighbor_IsWall(neighborIndex int) bool {
	return ((1 << neighborIndex) & node.neighborIsObstacle) != 0
}

// SetImpBTAWall 设置某个节点为墙
func (m *Map) SetImpBTAWall(x, y int) {
	if m.isMapNode(x, y) {
		m.Nodes[x][y].IsPOCWOC = Bit2                                          //直接设置并且覆盖其他 因为不能在寻路途中设置
		m.nowObstacleNodeSlice = append(m.nowObstacleNodeSlice, m.Nodes[x][y]) //额外:添加到尾部
		m.nowObstacleNodeDic[m.Nodes[x][y]] = len(m.nowObstacleNodeSlice) - 1  //额外:存储进墙dic v是障碍物在切片中的索引
	}
}

// SetImpBTARoad 设置道路
func (m *Map) SetImpBTARoad(x, y int) {
	if x >= 0 && x < m.Rows && y >= 0 && y < m.Cols {
		if m.Nodes[x][y].IsWall() { //必须判断是不是墙
			m.Nodes[x][y].IsPOCWOC = 0 //直接设置并且覆盖其他 因为不能在寻路途中设置
			//肯定有这个墙dic映射
			if index, ok := m.nowObstacleNodeDic[m.Nodes[x][y]]; ok {
				//1.尾部元素记录的索引先更新
				m.nowObstacleNodeDic[m.nowObstacleNodeSlice[len(m.nowObstacleNodeSlice)-1]] = index
				//2.交换当前和尾部
				m.nowObstacleNodeSlice[index], m.nowObstacleNodeSlice[len(m.nowObstacleNodeSlice)-1] = m.nowObstacleNodeSlice[len(m.nowObstacleNodeSlice)-1], m.nowObstacleNodeSlice[index]
				//end: 删除映射  删除尾部
				delete(m.nowObstacleNodeDic, m.Nodes[x][y])                                     //删除映射
				m.nowObstacleNodeSlice = m.nowObstacleNodeSlice[:len(m.nowObstacleNodeSlice)-1] //删除尾部
			}
		}
	}
}

// 检查节点是否有效
func (m *Map) isMapNode(x, y int) bool {
	return x >= 0 && x < m.Rows && y >= 0 && y < m.Cols
}

// DRA专用 检查当前索引是否是墙 --使用位图
func (m *Map) isWall_DynamicRayPathFind(x, y int) bool {
	return (m.Nodes_BitMap[x][y/8] & (128 >> (y % 8))) != 0 //不为0代表该位是1 1代表障碍 返回true
}

// ---------------------------------------Node
// 节点标记处于开启列表
func (node *Node) signInOpenList1() {
	//1.直接或运算标记当前节点处于开启列表
	node.IsPOCWOC |= Bit1 //0b 00000010
}

func (node *Node) signInOpenList2() {
	//1.直接或运算标记当前节点处于开启列表
	node.IsPOCWOC |= Bit4 //0b 00010000
}

// 节点标记处于开启列表
func (node *Node) signInCloseList1_ClearOpenFlag1() {
	//1.清除OpenFlag 与运算取反的 ^Bit1(1 << 1)
	//node.IsPOCWOC &= ^Bit1      //等同于下面
	node.IsPOCWOC &= 0b11111101 //0b 11111101
	//2.再或运算Bit0
	node.IsPOCWOC |= Bit0 //0b 00000001
}

// 节点标记处于开启列表
func (node *Node) signInCloseList2_ClearOpenFlag2() {
	//1.清除OpenFlag 与运算取反的 ^Bit1(1 << 1)
	//node.IsPOCWOC &= ^Bit4      //等同于下面
	node.IsPOCWOC &= 0b11101111 //0b 11111101
	//2.再或运算Bit0
	node.IsPOCWOC |= Bit3 //0b 00001000
}

// 节点标记拥有父节点
func (node *Node) signHasParent() {
	//Bit5
	node.IsPOCWOC |= Bit5 //0b 00100000
}

// 节点清除所有开启和关闭列表标记
func (node *Node) clearOpen_Close_Parent_SignFlag() {
	node.IsPOCWOC &= Bit2 // 0b 00000100
}

// IsWall 检查节点是否是墙
func (node *Node) IsWall() bool {
	return (node.IsPOCWOC & Bit2) != 0 //0b 00000100
}

func (node *Node) IsRoad() bool {
	return !node.IsWall()
}

// 检查节点是否在开启列表 (暂时给Jps使用 错误的示范 后面再改吧)
func (node *Node) isUsedToInOpenList() bool {
	return (node.IsPOCWOC & Bit1) != 0 //0b 00000010
}

// 检查节点当前是否在开启列表
func (node *Node) isInOpenList1() bool {
	return (node.IsPOCWOC & Bit1) != 0 //0b 00000010
}

// 检查节点当前是否在关闭列表
func (node *Node) isInCloseList1() bool {
	return (node.IsPOCWOC & Bit0) != 0 //0b 00000001
}

// 检查节点当前是否在开启列表
func (node *Node) isInOpenList2() bool {
	return (node.IsPOCWOC & Bit4) != 0 //0b 00010000
}

// 检查节点当前是否在关闭列表
func (node *Node) isInCloseList2() bool {
	return (node.IsPOCWOC & Bit3) != 0 //0b 00001000
}

// 检查节点当前是否在有父节点
func (node *Node) hasParent() bool {
	return (node.IsPOCWOC & Bit5) != 0 //0b 00001000
}

// 检查节点是否在关闭列表 不需要了
//func (node *Node) isInCloseList1() bool {
//	return node.IsPOCWOC&Bit2 != 0 //0b 00000100
//}

//var eightDirSlice = [][]int{
//	{-1, 0}, // left 上 //index 0
//	{0, 1},  // up 右  //index 1
//	{1, 0},  // right 下  //index 2
//	{0, -1}, // down 左 //index 3
//	//
//	{-1, 1},  //left up 右上 //index 4
//	{1, 1},   //right up 右下 //index 5
//	{1, -1},  //right down 左下  //index 6
//	{-1, -1}, //left down 左上 //index 7
//}

// -------------------------------------------------------------------------jpsBit
func (m *Map) calStartNodeMoveDir(startNode *Node) {
	startNodeX := startNode.X
	startNodeY := startNode.Y
	judgeNodeX, judgeNodeY := 0, 0
	//下一个方向点界内且不是墙 才给起点添加该方向
	for i := 0; i <= 7; i++ {
		//0.更新要判断的下一方向节点
		judgeNodeX = startNodeX + eightDirSlice[i][0]
		judgeNodeY = startNodeY + eightDirSlice[i][1]
		//1.阻断界外
		if !m.isMapNode(judgeNodeX, judgeNodeY) {
			continue
		}
		//1.上下左右直向
		if !m.Nodes[judgeNodeX][judgeNodeY].IsWall() {
			switch i {
			//上 右 下 左
			case 0:
				startNode.moveDir |= Bit0
			case 1:
				startNode.moveDir |= Bit1
			case 2:
				startNode.moveDir |= Bit2
			case 3:
				startNode.moveDir |= Bit3
			//右上 右下 左下 左上 有1个不是墙 那么添加该方向
			case 4:
				if !m.Nodes[startNodeX-1][startNodeY].IsWall() || !m.Nodes[startNodeX][startNodeY+1].IsWall() {
					startNode.moveDir |= Bit4
				}
			case 5:
				if !m.Nodes[startNodeX+1][startNodeY].IsWall() || !m.Nodes[startNodeX][startNodeY+1].IsWall() {
					startNode.moveDir |= Bit5
				}
			case 6:
				if !m.Nodes[startNodeX+1][startNodeY].IsWall() || !m.Nodes[startNodeX][startNodeY-1].IsWall() {
					startNode.moveDir |= Bit6
				}
			case 7:
				if !m.Nodes[startNodeX-1][startNodeY].IsWall() || !m.Nodes[startNodeX][startNodeY-1].IsWall() {
					startNode.moveDir |= Bit7
				}
			}
		}
	}
}

// A星对open弹出的节点进行根父节点可视化
func (m *Map) lazyRayCastNodeForAstar(fatherNode, startNode *Node) {
	var tmpRootFatherNode, availableRootFatherNode *Node = nil, nil
	if fatherNode != startNode {
		needChangeParent := false
		//1.尝试回溯获得根父节点
		if fatherNode.Parent.Parent != nil { //
			tmpRootFatherNode = fatherNode.Parent.Parent
		}
		//2.尝试判断新根节点连通性
		for tmpRootFatherNode != nil {
			if m.JudgeLineObstacleNew(fatherNode.X, fatherNode.Y, tmpRootFatherNode.X, tmpRootFatherNode.Y) {
				needChangeParent = true
				availableRootFatherNode = tmpRootFatherNode //先更新可用根父节点为第2...n级
				//tmpRootFatherNode = tmpRootFatherNode.Parent //尝试下1级根父节点 如果下1级不可以直连 那么availableRootFatherNode记录的就是最后一次可直连的根父
			}
			//就算不能与当前根父节点相连 也要一直回溯 直到没有根父节点
			tmpRootFatherNode = tmpRootFatherNode.Parent
		}
		//3.判断是否改变根父节点 不是Fix 因为已经不在open中了 不需要判断是否小于原来的F 因为当前新F一定 <= 原来的F,不需要Fix则无事发生
		if needChangeParent {
			newG := availableRootFatherNode.G + m.euclideanDistance(fatherNode.X, fatherNode.Y, availableRootFatherNode.X, availableRootFatherNode.Y)
			newF := newG //newF没有启发函数
			//2.注意 g也要更新 根父节点的g + 本身到新根父节点的g
			fatherNode.G = newG
			//3.更新为更小的F
			fatherNode.F = newF
			//4.更新为新的根父节点
			fatherNode.Parent = availableRootFatherNode
		}
	}
}

// jps对open弹出的节点进行根父节点可视化 --与A星不同,由于jps的视野范围大多不是相邻节点,或者说父类距离较远,正常的可视化根节点还是不准确
// 要同最后1个不相连的根父节点重现中间点,从最远的中间点开始判断与当前open点是否相连,相连立即停止,这个最远的中间点就是新的父节点
// 如果这个新中间父节点本身没有父节点 那么不相连的根父节点就是这个父节点的父节点,父节点计算新的F,父节点拥有新的根父节点,跳点的F和父节点直接改变不用比较
// 如果这个新中间父节点本身有父节点 那么直接计算更改跳点新F和新父节点就行
// 如果没有新中间父节点可以与跳点相连,那么上1个根父节点就是最远的新父节点,根父节点本身必定有自己的父节点,所以直接计算更改跳点的F和新父节点就行
// 全程没有改变根父节点的父节点,没有父节点的中间父节点会被增加新的父节点(就是不相连的根父节点)
func (m *Map) lazyRayCastNodeForJps(openNode, startNode *Node) {
	//open点不是终点时
	if openNode != startNode {
		var tmpRootFatherNode, availableRootFatherNode *Node = nil, nil
		var rootSon, thisrootSonFather *Node = nil, nil //记录根父节点的子节点--用于重现中间父节点
		var isRoot, isMid = false, false                //互斥
		//1.尝试回溯获得根父节点
		if openNode.Parent.Parent != nil { //
			tmpRootFatherNode = openNode.Parent.Parent
			rootSon = openNode.Parent
		}
		//2.尝试判断新根节点连通性
		for tmpRootFatherNode != nil {
			//先与根父节点判断是否相连
			if m.JudgeLineObstacleNew(openNode.X, openNode.Y, tmpRootFatherNode.X, tmpRootFatherNode.Y) {
				isRoot = true
				isMid = false
				availableRootFatherNode = tmpRootFatherNode //先更新可用根父节点为第2...n级
				//tmpRootFatherNode = tmpRootFatherNode.Parent //尝试下1级根父节点 如果下1级不可以直连 那么availableRootFatherNode记录的就是最后一次可直连的根父
			} else { //与根父节点不相连 重现 rootSon 与 tmpRootFatherNode 的中间父节点 循环判断相连
				m.OriginalNodeSlice = m.OriginalNodeSlice[:0]
				m.OriginalNodeSlice = append(m.OriginalNodeSlice, tmpRootFatherNode, rootSon, openNode)
				//1.按顺序生成中间像素点 tmpRootFatherNode -> rootSon
				m.GetMidNotObstacleFromX1ToX2ForStep2(tmpRootFatherNode.X, tmpRootFatherNode.Y, rootSon.X, rootSon.Y, m.OriginalNodeSlice, &m.SMNodeSlice)
				//2.有可以与openNode相连的中间点 马上结束
				for i := 0; i < len(m.SMNodeSlice); i++ {
					if m.JudgeLineObstacleNew(m.SMNodeSlice[i].X, m.SMNodeSlice[i].Y, openNode.X, openNode.Y) {
						isRoot = false
						isMid = true
						thisrootSonFather = tmpRootFatherNode
						availableRootFatherNode = m.SMNodeSlice[i]
						break
					}
				}
				////3.不管有没有符合中间点 都结束整个回溯逻辑 因为与这次根父节点不相连 没必要继续判断下一个根父节点
				//break
			}
			//3.就算不能与当前根父节点相连或者中间父节点 也要一直回溯 直到没有根父节点
			rootSon = tmpRootFatherNode //当前根父节点更新为旧孩子父节点
			tmpRootFatherNode = tmpRootFatherNode.Parent
		}
		//3.判断是否改变根父节点 不是Fix 因为已经不在open中了 不需要判断是否小于原来的F 因为当前新F一定 <= 原来的F,不需要Fix则无事发生
		if isRoot { //当前open与根父节点相连
			newG := availableRootFatherNode.G + m.euclideanDistance(openNode.X, openNode.Y, availableRootFatherNode.X, availableRootFatherNode.Y)
			newF := newG //newF没有启发函数
			//2.注意 g也要更新 根父节点的g + 本身到新根父节点的g
			openNode.G = newG
			//3.更新为更小的F
			openNode.F = newF
			//4.更新为新的根父节点
			openNode.Parent = availableRootFatherNode
		} else if isMid { //当前open与中间父节点相连
			//1.如果中间父节点本身有父节点 那么不动它
			if availableRootFatherNode.Parent != nil {
				newG := availableRootFatherNode.G + m.euclideanDistance(openNode.X, openNode.Y, availableRootFatherNode.X, availableRootFatherNode.Y)
				newF := newG //newF没有启发函数
				//2.注意 g也要更新 根父节点的g + 本身到新根父节点的g
				openNode.G = newG
				//3.更新为更小的F
				openNode.F = newF
				//4.更新为新的根父节点
				openNode.Parent = availableRootFatherNode
			} else { //2.如果中间父节点本身没有父节点 要用判断成功记录的重现源节点 thisrootSonFather 当做父节点 源节点可能是起点 --肯定有GF
				thisRootSonG := thisrootSonFather.G + m.euclideanDistance(thisrootSonFather.X, thisrootSonFather.Y, availableRootFatherNode.X, availableRootFatherNode.Y)
				newG := thisRootSonG + m.euclideanDistance(availableRootFatherNode.X, availableRootFatherNode.Y, openNode.X, openNode.Y)
				newF := newG
				//2.注意 g也要更新 根父节点的g + 本身到新根父节点的g
				openNode.G = newG
				//3.更新为更小的F
				openNode.F = newF
				//4.openNode更新为新的中间父节点
				openNode.Parent = availableRootFatherNode
				//5.中间父节点本身也要加入新的父节点 记录GF 然后打标记
				availableRootFatherNode.G = thisRootSonG
				availableRootFatherNode.F = thisRootSonG
				availableRootFatherNode.Parent = thisrootSonFather
				availableRootFatherNode.signHasParent()
				m.hasParentList = append(m.hasParentList, availableRootFatherNode)
			}
		}
	}
}

func (m *Map) jpsBitPathFind(startX, startY, endX, endY int) bool {
	//计算必要信息
	startNode := m.Nodes[startX][startY]
	endNode := m.Nodes[endX][endY]
	m.startNode = startNode
	m.endNode = endNode
	m.popTimes = 1
	//
	startNode.G = 0
	startNode.F = 0
	startNode.signInOpenList1()      // 标记open
	m.calStartNodeMoveDir(startNode) // 计算起点移动方向(已处理)
	Push(&m.openList1, startNode)    // 修改：传递指针，无返回值赋值
	m.pushTimes++
	//用固定的根父节点 区分 不断改变的父节点
	var originalNode *Node
	var moveDir uint8
	//Loop 判断当前新的FatherNode
	for len(m.openList1) > 0 {
		//1.弹出 添加进closeList
		originalNode = Pop(&m.openList1)
		//fmt.Println(originalNode.X, " ", originalNode.Y)
		m.closeList1 = append(m.closeList1, originalNode)
		originalNode.signInCloseList1_ClearOpenFlag1()
		m.popTimes++
		//2.Lazy Ray cast --不会对起点操作
		//m.lazyRayCastNodeForAstar(originalNode, startNode)
		//3.判断是否是终点 直接回溯路径
		if originalNode == m.endNode {
			m.oneWayRetracePath(m.startNode, originalNode)
			return true
		}
		moveDir = originalNode.moveDir
		//4.解析存储的8方向
		//直向
		if moveDir&Bit0 != 0 { //上
			m.get_Up_VerticalRealJumpPoint(originalNode.Y+1 < m.Cols, originalNode.Y-1 >= 0, false, originalNode)
		}
		if moveDir&Bit1 != 0 { //右
			m.get_Right_HorizontalRealJumpPoint(originalNode.X+1 < m.Rows, originalNode.X-1 >= 0, false, originalNode)
		}
		if moveDir&Bit2 != 0 { //下
			m.get_Down_VerticalRealJumpPoint(originalNode.Y+1 < m.Cols, originalNode.Y-1 >= 0, false, originalNode)
		}
		if moveDir&Bit3 != 0 { //左
			m.get_Left_HorizontalRealJumpPoint(originalNode.X+1 < m.Rows, originalNode.X-1 >= 0, false, originalNode)
		}
		//斜向
		if moveDir&Bit4 != 0 { //右上
			m.get_RightUp_DiagonalRealJumpPoint(originalNode)
		}
		if moveDir&Bit5 != 0 { //右下
			m.get_RightDown_DiagonalRealJumpPoint(originalNode)
		}
		if moveDir&Bit6 != 0 { //左下
			m.get_LeftDown_DiagonalRealJumpPoint(originalNode)
		}
		if moveDir&Bit7 != 0 { //左上
			m.get_LeftUp_DiagonalRealJumpPoint(originalNode)
		}
	}
	//默认寻路失败
	fmt.Println("开启列表为空 路径查询失败 请排查！")
	return false
}

// push open使用
func (m *Map) activeRayCastNode(fatherNode, neighbor *Node, alreadyHasParent, needAddInOpen bool, newMoveDir uint8) *Node {
	//================不管在不在open 都要与根父作一次可视化检查 1级是当前父节点(上面判断过了 肯定可以与所有可用邻居直连) 2级是当前父节点的父节点(根父节点)
	//1.注意当前邻居如果不在open中 是没有父节点的 都假设可用父节点就是当前fatherNode
	//2.注意当前邻居如果在open中 必定有自己的父节点 都假设可用父节点就是当前fatherNode
	var tmpRootFatherNode, availableRootFatherNode *Node
	availableRootFatherNode = fatherNode
	//1.尝试回溯根父节点 当前fatherNode首次进来是起点的情况 那么它的邻居肯定都不在open中
	if fatherNode.Parent != nil { //
		tmpRootFatherNode = fatherNode.Parent
	}
	//2.尝试判断新根节点连通性
	for tmpRootFatherNode != nil {
		if m.JudgeLineObstacleNew(neighbor.X, neighbor.Y, tmpRootFatherNode.X, tmpRootFatherNode.Y) {
			availableRootFatherNode = tmpRootFatherNode  //先更新可用根父节点为第2...n级
			tmpRootFatherNode = tmpRootFatherNode.Parent //尝试下1级根父节点 如果下1级不可以直连 那么availableRootFatherNode记录的就是最后一次可直连的根父
		} else {
			break
		}
	}
	//3.可视化检测结束 当前可用父可能是 fatherNode本身 或者 2级以上根父
	//newG = 当前邻居到可用根父节点的G(直线欧几里得距离)
	newG := availableRootFatherNode.G + m.euclideanDistance(availableRootFatherNode.X, availableRootFatherNode.Y, neighbor.X, neighbor.Y)
	//newF = 可用根父节点本身的G(这里等于fatherNode的F) + newG
	newF := newG
	//-----------------------------------------------------------邻居判断在不在open中
	//如果邻居已经存在OpenList(节点只会有3种情况 不在open也不在close 只在open 只在close) 如果当前计算出来的新F值小于 自己 当前已存在Open中的F值 那么替换为新的F 并且Fix维护堆性质
	if neighbor.isInOpenList1() { //邻居在open中 已经有自己的父节点
		//对于当前处于open中的邻居 都是新的父节点 与 自己本身的代价进行比较
		//2.用来与自己旧的F对比 如果比之前的小就替换 否则什么都不做直接跳过
		if newF < neighbor.F {
			//2.注意 g也要更新
			//2.更新新的 上面的迪杰g 等同于 newF
			neighbor.G = newG
			//3.更新为更小的F
			neighbor.F = newF
			//4.更新为新的根父节点
			neighbor.Parent = availableRootFatherNode
			//4.Fix open,索引就是neighbor节点存储的openListIndex
			Fix(&m.openList1, neighbor.IndexInOpenList)
			m.FixTimes++
		}
	} else {
		//可视化检测结束 当前可用父可能是 fatherNode本身 或者 2级以上根父
		if alreadyHasParent { //不在open 但是有父节点(剪枝点)
			if newF < neighbor.F {
				//2.注意 g也要更新
				//2.更新新的 上面的迪杰g 等同于 newF
				neighbor.G = newG
				//3.更新为更小的F
				neighbor.F = newF
				//4.更新为新的根父节点
				neighbor.Parent = availableRootFatherNode
			}
		} else { //不在open 也没有父节点
			//1.对于当前不处于open中的邻居 邻居本身没有父节点 不需要比较旧的邻居代价 直接计算当前新的父或者根父节点代价存储 并添加进open
			//2.注意 g也要更新
			//2.更新新的 上面的迪杰g 等同于 newF
			neighbor.G = newG
			//3.更新为更小的F
			neighbor.F = newF
			//4.更新为新的根父节点
			neighbor.Parent = availableRootFatherNode
		}
		//通过外部传入是否需要添加进open
		if needAddInOpen {
			//5.Push openList1
			Push(&m.openList1, neighbor)
			//6.记得标记
			neighbor.signInOpenList1()
			m.pushTimes++
		}
	}
	//End
	neighbor.moveDir |= newMoveDir                      //不管怎么回溯 都要增加新的方向
	neighbor.signHasParent()                            //不管如何 邻居只要调用了回溯方法 都会有父节点
	m.hasParentList = append(m.hasParentList, neighbor) //我说了 一定记得添加记录！
	return neighbor
}

// checkDiagonalToHandVJumpNode_DeepBestNew jumpNode即为a星中的neibor
func (m *Map) checkJumpNode_New(newFatherNode, jumpNode *Node, newMoveDir uint8) (*Node, bool) {
	//1.先看跳点在不在close中 如果在 像A星一样直接退出逻辑不管
	if jumpNode.isInCloseList1() {
		return jumpNode, false
	}
	//2.在open或者不在open中
	//================不管在不在open 都要与根父作一次可视化检查 1级是当前父节点(上面判断过了 肯定可以与所有可用邻居直连) 2级是当前父节点的父节点(根父节点)
	//1.注意当前邻居如果不在open中 是没有父节点的 都假设可用父节点就是当前fatherNode
	//2.注意当前邻居如果在open中 必定有自己的父节点 都假设可用父节点就是当前fatherNode
	var tmpRootFatherNode, availableRootFatherNode *Node = nil, newFatherNode //当前fatherNode默认为最开始的可用根父节点
	//1.尝试回溯根父节点 当前fatherNode首次进来是起点的情况 那么它的邻居肯定都不在open中
	if newFatherNode.Parent != nil { //
		tmpRootFatherNode = newFatherNode.Parent
	}
	//2.尝试判断新根节点连通性
	for tmpRootFatherNode != nil {
		if m.JudgeLineObstacleNew(jumpNode.X, jumpNode.Y, tmpRootFatherNode.X, tmpRootFatherNode.Y) {
			availableRootFatherNode = tmpRootFatherNode //先更新可用根父节点为第2...n级
		}
		//不管能不能相连 继续回溯根父节点 直到根父节点为空
		tmpRootFatherNode = tmpRootFatherNode.Parent
	}
	//3.可视化检测结束 当前可用父可能是 fatherNode本身 或者 2级以上根父
	//newG = 当前邻居到可用根父节点的G(直线欧几里得距离) newF = 可用根父节点本身的G(这里等于fatherNode的F) + newG
	newG := availableRootFatherNode.G + m.euclideanDistance(availableRootFatherNode.X, availableRootFatherNode.Y, jumpNode.X, jumpNode.Y)
	newF := newG
	//4.跳点在open中 对比新旧F更新跳点父节点
	if jumpNode.isInOpenList1() { //在open则比对新父计算的newF和旧的存储的F
		if newF < jumpNode.F {
			jumpNode.G = newG
			jumpNode.F = newF
			//更新父节点为代价更小的根父节点
			jumpNode.Parent = availableRootFatherNode
			Fix(&m.openList1, jumpNode.IndexInOpenList)
			m.FixTimes++
		} //否则jumpNode本身无事发生
	} else { //跳点不在open 直接设置新的可能代价更小的根父节点
		jumpNode.G = newG
		jumpNode.F = newF
		jumpNode.Parent = availableRootFatherNode
		Push(&m.openList1, jumpNode)
		jumpNode.signInOpenList1()
		m.pushTimes++
		//4.只有不在open中的跳点才可以添加新的方向
		jumpNode.moveDir |= newMoveDir
		jumpNode.isOut = true
	}
	//5.不管怎么样都返回跳点
	return jumpNode, true
}

func (m *Map) addJumpNode_ToOpen_New(newFatherNode, jumpNode *Node, newMoveDir uint8) (*Node, bool) {
	//2.在open或者不在open中
	//================不管在不在open 都要与根父作一次可视化检查 1级是当前父节点(上面判断过了 肯定可以与所有可用邻居直连) 2级是当前父节点的父节点(根父节点)
	//1.注意当前邻居如果不在open中 是没有父节点的 都假设可用父节点就是当前fatherNode
	//2.注意当前邻居如果在open中 必定有自己的父节点 都假设可用父节点就是当前fatherNode
	var tmpRootFatherNode, availableRootFatherNode *Node = nil, newFatherNode //当前fatherNode默认为最开始的可用根父节点
	//1.尝试回溯根父节点 当前fatherNode首次进来是起点的情况 那么它的邻居肯定都不在open中
	if newFatherNode.Parent != nil { //
		tmpRootFatherNode = newFatherNode.Parent
	}
	//2.尝试判断新根节点连通性
	for tmpRootFatherNode != nil {
		if m.JudgeLineObstacleNew(jumpNode.X, jumpNode.Y, tmpRootFatherNode.X, tmpRootFatherNode.Y) {
			availableRootFatherNode = tmpRootFatherNode //先更新可用根父节点为第2...n级
		}
		//不管能不能相连 继续回溯根父节点 直到根父节点为空
		tmpRootFatherNode = tmpRootFatherNode.Parent
	}
	//3.可视化检测结束 当前可用父可能是 fatherNode本身 或者 2级以上根父
	//newG = 当前邻居到可用根父节点的G(直线欧几里得距离) newF = 可用根父节点本身的G(这里等于fatherNode的F) + newG
	newG := availableRootFatherNode.G + m.euclideanDistance(availableRootFatherNode.X, availableRootFatherNode.Y, jumpNode.X, jumpNode.Y)
	newF := newG
	//4.跳点在open中 对比新旧F更新跳点父节点
	jumpNode.G = newG
	jumpNode.F = newF
	jumpNode.Parent = availableRootFatherNode
	Push(&m.openList1, jumpNode)
	jumpNode.signInOpenList1()
	m.pushTimes++
	//4.只有不在open中的跳点才可以添加新的方向
	jumpNode.moveDir |= newMoveDir
	//5.不管怎么样都返回跳点
	jumpNode.isOut = true
	return jumpNode, true
}

// 专门处理已经处于open中的跳点 被新的父节点重复扫描
func (m *Map) fixRayCastNode_InOpen_New(openNode, newFatherNode *Node) {
	//1.当前邻居确认在open中 必定有自己的父节点 都假设可用父节点就是当前fatherNode
	var tmpRootFatherNode, availableRootFatherNode *Node = nil, newFatherNode //当前fatherNode默认为最开始的可用根父节点
	//2.尝试回溯根父节点 当前fatherNode首次进来是起点的情况 那么它的邻居肯定都不在open中
	if newFatherNode.Parent != nil { //
		tmpRootFatherNode = newFatherNode.Parent
	}
	//3.尝试判断新根节点连通性
	for tmpRootFatherNode != nil {
		if m.JudgeLineObstacleNew(openNode.X, openNode.Y, tmpRootFatherNode.X, tmpRootFatherNode.Y) {
			availableRootFatherNode = tmpRootFatherNode //先更新可用根父节点为第2...n级
		}
		//不管能不能相连 继续回溯根父节点 直到根父节点为空
		tmpRootFatherNode = tmpRootFatherNode.Parent
	}
	//4.可视化检测结束 当前可用父可能是 fatherNode本身 或者 2级以上根父
	//newG = 当前邻居到可用根父节点的G(直线欧几里得距离) newF = 可用根父节点本身的G(这里等于fatherNode的F) + newG
	newG := availableRootFatherNode.G + m.euclideanDistance(availableRootFatherNode.X, availableRootFatherNode.Y, openNode.X, openNode.Y)
	newF := newG
	//5.跳点确认在open中 对比新旧F更新跳点父节点
	if newF < openNode.F {
		openNode.G = newG
		openNode.F = newF
		//更新父节点为代价更小的根父节点
		openNode.Parent = availableRootFatherNode
		Fix(&m.openList1, openNode.IndexInOpenList)
		m.FixTimes++
	}
}

func (m *Map) calHEndPointLoopTimes(fatherNode *Node, isRight bool) int {
	if m.endNode.X == fatherNode.X {
		if isRight { //右移动
			if m.endNode.Y >= fatherNode.Y { //移动方向正确
				return (m.endNode.Y / 8) - (fatherNode.Y / 8) //计算当前位图到终点所在的位图需要的遍历的循环次数
			}
			return -1
		} else {
			if m.endNode.Y <= fatherNode.Y { //移动方向正确
				return (fatherNode.Y / 8) - (m.endNode.Y / 8) //计算当前位图到终点所在的位图需要的遍历的循环次数
			}
			return -1
		}
	}
	//默认不符合条件
	return -1
}

func (m *Map) calVEndPointLoopTimes(fatherNode *Node, isDown bool) int {
	if m.endNode.Y == fatherNode.Y {
		if isDown {
			if m.endNode.X >= fatherNode.X { //移动方向正确
				return (m.endNode.X / 8) - (fatherNode.X / 8) //计算当前位图到终点所在的位图需要的遍历的循环次数
			}
			return -1
		} else {
			if m.endNode.X <= fatherNode.X { //移动方向正确
				return (fatherNode.X / 8) - (m.endNode.X / 8) //计算当前位图到终点所在的位图需要的遍历的循环次数
			}
			return -1
		}
	}
	//默认不符合条件
	return -1
}

func (m *Map) isMapNodeAndNotWall00(x, y int) (*Node, bool) {
	if m.isMapNode(x, y) && !m.Nodes[x][y].IsWall() {
		return m.Nodes[x][y], true
	}
	return nil, false
}

func (m *Map) isMapNodeAndNotWall01(x, y int) bool {
	return m.isMapNode(x, y) && !m.Nodes[x][y].IsWall()
}

// 参数 注意 当前changeX或changeY有可能是上一个位图的末尾 也就是说jumpNode有可能就是这个点 这个点有可能与fatherNode重合 --在这里就把跳点处理了
func (m *Map) doHMayEndPointBitJumpNode(MStartThenChangeX, MStartThenChangeY int, fatherNode *Node, deltaY int, availableP, availableN, fatherIsDiagonal bool) (*Node, bool) {
	changeX, changeY := MStartThenChangeX, MStartThenChangeY
	var newMoveDir uint8 = 0
	var jumpNode, nextNode *Node = nil, nil
	var realP, realN, jumpNodeIsMapNodeAndNotWall, nextNodeIsMapNodeAndNotWall = false, false, false, false
	//该位全量遍历 因为不知道障碍连续性 所以不适用部分遍历
	//LOOP
	for {
		//1.当前点是不是终点 是的话把终点当做跳点
		if jumpNode, jumpNodeIsMapNodeAndNotWall = m.isMapNodeAndNotWall00(changeX, changeY); jumpNodeIsMapNodeAndNotWall {
			//先看当前点是不是终点 终点也不需要方向了
			if jumpNode == m.endNode { //找到终点后 停止整个逻辑 因为终于以后的任何遍历必定拉长路径
				if fatherIsDiagonal { //父节点是斜向点 父节点不处于open或close中
					if jumpNode.isInOpenList1() { //终点跳点在open 终点不可能在close中被这里发现 因为终点弹出时已经结束寻路回溯路径 所以停止
						return nil, false
					} else { //终点强制邻居不在open 也就是第一次被发现 但是跳点是斜向点本身 不是强制邻居 所以返回nil,找到有效强制邻居true
						return nil, true
					}
				} else { //父节点不是斜向点 父节点是一级源点 父节点必定处于close中
					if jumpNode.isInOpenList1() { //终点在open 尝试fix 停止查找
						m.fixRayCastNode_InOpen_New(jumpNode, fatherNode)
					} else { //跳点不在open也不在close 找到有效跳点
						m.addJumpNode_ToOpen_New(fatherNode, jumpNode, newMoveDir)
					}
					return jumpNode, true
				}
			}
			//再看是不是与父节点重合 重合则跳过
			if jumpNode == fatherNode {
				//End:下一个索引
				changeY += deltaY
				continue
			}
			//再看当前点是不是普通跳点
			//switch delta available//下一个点在范围内且不是障碍 那么判断跳点的时候障碍也在范围内
			if nextNode, nextNodeIsMapNodeAndNotWall = m.isMapNodeAndNotWall00(jumpNode.X, jumpNode.Y+deltaY); nextNodeIsMapNodeAndNotWall {
				if availableP && availableN {
					realP = m.Nodes[jumpNode.X+1][jumpNode.Y].IsWall() && !m.Nodes[nextNode.X+1][nextNode.Y].IsWall() //P
					realN = m.Nodes[jumpNode.X-1][jumpNode.Y].IsWall() && !m.Nodes[nextNode.X-1][nextNode.Y].IsWall() //N
				} else if availableP {
					realP = m.Nodes[jumpNode.X+1][jumpNode.Y].IsWall() && !m.Nodes[nextNode.X+1][nextNode.Y].IsWall() //P
				} else if availableN {
					realN = m.Nodes[jumpNode.X-1][jumpNode.Y].IsWall() && !m.Nodes[nextNode.X-1][nextNode.Y].IsWall() //N
				}
				//有符合的跳点 再次判断跳点在不在open 或 close中 在的话跳过
				if realP || realN {
					if fatherIsDiagonal { //父节点是斜向点 父节点不处于open或close中
						if jumpNode.isInOpenList1() || jumpNode.isInCloseList1() { //跳点在open  跳过该点 继续下一个
							realP, realN = false, false //重置找到跳点标志
						} else { //否则强制邻居生效 但是跳点是斜向点本身 不是强制邻居 返回nil,找到有效强制邻居true
							return nil, true
						}
					} else { //父节点不是斜向点 父节点是一级源点 父节点必定处于close中
						if jumpNode.isInOpenList1() { //跳点在open 尝试fix 跳过该点 继续下一个
							m.fixRayCastNode_InOpen_New(jumpNode, fatherNode)
							realP, realN = false, false //重置找到跳点标志
						} else if jumpNode.isInCloseList1() { //跳点在close 跳过
							realP, realN = false, false //重置找到跳点标志
						} else { //跳点不在open也不在close 找到有效跳点
							if deltaY > 0 { //原方向 右
								if realP {
									newMoveDir |= Bit1 | Bit5 //右下
								}
								if realN {
									newMoveDir |= Bit1 | Bit4 //右上
								}
							} else { //原方向 左
								if realP {
									newMoveDir |= Bit3 | Bit6 //左下
								}
								if realN {
									newMoveDir |= Bit3 | Bit7 //左上
								}
							}
							m.addJumpNode_ToOpen_New(fatherNode, jumpNode, newMoveDir)
							return nil, true //找到跳点 上面已经处理了 不返回跳点
						}
					}
				}
			} else {
				break
			}
			//End:下一个索引
			changeY += deltaY
		} else {
			break
		}
	}
	//默认终点位图没有符合条件的跳点
	return nil, false
}

// 参数 当前8位图起始M点 当前8位图需要遍历的次数(就是P或N最小的前导0个数) 当前遍历移动的XY差值
func (m *Map) doVMayEndPointBitJumpNode(MStartThenChangeX, MStartThenChangeY int, fatherNode *Node, deltaX int, availableP, availableN, fatherIsDiagonal bool) (*Node, bool) {
	changeX, changeY := MStartThenChangeX, MStartThenChangeY
	var newMoveDir uint8 = 0
	var jumpNode, nextNode *Node = nil, nil
	var realP, realN, jumpNodeIsMapNodeAndNotWall, nextNodeIsMapNodeAndNotWall = false, false, false, false
	//该位全量遍历 因为不知道障碍连续性 所以不适用部分遍历
	//LOOP
	for {
		//1.当前点是不是终点 是的话把终点当做跳点
		if jumpNode, jumpNodeIsMapNodeAndNotWall = m.isMapNodeAndNotWall00(changeX, changeY); jumpNodeIsMapNodeAndNotWall {
			//先看当前点是不是终点 终点也不需要方向了
			if jumpNode == m.endNode {
				if fatherIsDiagonal { //父节点是斜向点 父节点不处于open或close中
					if jumpNode.isInOpenList1() { //终点跳点在open 终点不可能在close中被这里发现 因为终点弹出时已经结束寻路回溯路径 所以停止
						return nil, false
					} else { //终点强制邻居不在open 也就是第一次被发现 但是跳点是斜向点本身 不是强制邻居 所以返回nil,找到有效强制邻居true
						return nil, true
					}
				} else { //父节点不是斜向点 父节点是一级源点 父节点必定处于close中
					if jumpNode.isInOpenList1() { //终点在open 尝试fix 停止查找
						m.fixRayCastNode_InOpen_New(jumpNode, fatherNode)
					} else { //跳点不在open也不在close 找到有效跳点
						m.addJumpNode_ToOpen_New(fatherNode, jumpNode, newMoveDir)
					}
					return jumpNode, true
				}
			}
			//再看是不是与父节点重合 重合则跳过
			if jumpNode == fatherNode {
				//End:下一个索引
				changeX += deltaX
				continue
			}
			//再看当前点是不是普通跳点
			//switch delta available 下一个点在范围内且不是障碍 那么障碍点判断也在范围内
			if nextNode, nextNodeIsMapNodeAndNotWall = m.isMapNodeAndNotWall00(jumpNode.X+deltaX, jumpNode.Y); nextNodeIsMapNodeAndNotWall {
				if availableP && availableN {
					realP = m.Nodes[jumpNode.X][jumpNode.Y+1].IsWall() && !m.Nodes[nextNode.X][nextNode.Y+1].IsWall() //P
					realN = m.Nodes[jumpNode.X][jumpNode.Y-1].IsWall() && !m.Nodes[nextNode.X][nextNode.Y-1].IsWall() //N
				} else if availableP {
					realP = m.Nodes[jumpNode.X][jumpNode.Y+1].IsWall() && !m.Nodes[nextNode.X][nextNode.Y+1].IsWall() //P
				} else if availableN {
					realN = m.Nodes[jumpNode.X][jumpNode.Y-1].IsWall() && !m.Nodes[nextNode.X][nextNode.Y-1].IsWall() //N
				}
				//有符合的跳点就提前退出
				//有符合的跳点 再次判断跳点在不在open 或 close中 在的话跳过
				if realP || realN {
					if fatherIsDiagonal { //父节点是斜向点 父节点不处于open或close中
						if jumpNode.isInOpenList1() || jumpNode.isInCloseList1() { //跳点在open  跳过该点 继续下一个
							realP, realN = false, false //重置找到跳点标志
						} else { //否则强制邻居生效 但是跳点是斜向点本身 不是强制邻居 返回nil,找到有效强制邻居true
							return nil, true
						}
					} else { //父节点不是斜向点 父节点是一级源点 父节点必定处于close中
						if jumpNode.isInOpenList1() { //跳点在open 尝试fix 跳过该点 继续下一个
							m.fixRayCastNode_InOpen_New(jumpNode, fatherNode)
							realP, realN = false, false //重置找到跳点标志
						} else if jumpNode.isInCloseList1() { //跳点在close 跳过
							realP, realN = false, false //重置找到跳点标志
						} else { //跳点不在open也不在close 找到有效跳点
							if deltaX > 0 { //原方向 下
								if realP {
									newMoveDir |= Bit2 | Bit5 //右下
								}
								if realN {
									newMoveDir |= Bit2 | Bit6 //左下
								}
							} else { //原方向 上
								if realP {
									newMoveDir |= Bit0 | Bit4 //右上
								}
								if realN {
									newMoveDir |= Bit0 | Bit7 //左上
								}
							}
							m.addJumpNode_ToOpen_New(fatherNode, jumpNode, newMoveDir)
							return nil, true //找到跳点 上面已经处理了 不返回跳点
						}
					}
				}
			} else {
				break
			}
			//End:下一个索引
			changeX += deltaX
		} else {
			break
		}
	}
	//默认终点位图没有符合条件的跳点
	return nil, false
}

//if PLastIsObstacle && NLastIsObstacle {
//smallerPorNTimes = (128 >> calLeadingOne_8bit(nowBitP)) | (128 >> calLeadingOne_8bit(nowBitN))
//} else if PLastIsObstacle {
//smallerPorNTimes = (128 >> calLeadingOne_8bit(nowBitP)) | ((nowBitN >> 1) & ^nowBitN)
//} else if NLastIsObstacle {
//smallerPorNTimes = (128 >> calLeadingOne_8bit(nowBitN)) | ((nowBitP >> 1) & ^nowBitP)
//} else {
//smallerPorNTimes = calLeadingZero_8bit(((nowBitP >> 1) & ^nowBitP) | ((nowBitN >> 1) & ^nowBitN))
//}

func (m *Map) reset_Right_TemData(temJumpNode *Node, fatherX, fatherY, bitStartIndex, mayEndPointLoopTimes, loopTimes *int, nowBitN, nowBitM, nowBitP *uint8, PLastIsObstacle, NLastIsObstacle, mayFindJumpNode *bool, availableN, availableP bool, bitMap [][]uint8) {
	*fatherX, *fatherY = temJumpNode.X, temJumpNode.Y
	*bitStartIndex = *fatherY / 8
	if availableP {
		*nowBitP = clearLeftBits(bitMap[*fatherX+1][*bitStartIndex], (*fatherY%8)+1)
	}
	if availableN {
		*nowBitN = clearLeftBits(bitMap[*fatherX-1][*bitStartIndex], (*fatherY%8)+1)
	}
	*nowBitM = clearLeftBits(bitMap[*fatherX][*bitStartIndex], (*fatherY%8)+1)
	*PLastIsObstacle = false
	*NLastIsObstacle = false
	*mayFindJumpNode = false
	*mayEndPointLoopTimes = m.calHEndPointLoopTimes(temJumpNode, true)
	*loopTimes = 0
}

func (m *Map) reset_Left_TemData(temJumpNode *Node, fatherX, fatherY, bitStartIndex, mayEndPointLoopTimes, loopTimes *int, nowBitN, nowBitM, nowBitP *uint8, PLastIsObstacle, NLastIsObstacle, mayFindJumpNode *bool, availableN, availableP bool, bitMap [][]uint8) {
	*fatherX, *fatherY = temJumpNode.X, temJumpNode.Y
	*bitStartIndex = *fatherY / 8
	if availableP {
		*nowBitP = clearRightBits(bitMap[*fatherX+1][*bitStartIndex], 8-(*fatherY%8))
	}
	if availableN {
		*nowBitN = clearRightBits(bitMap[*fatherX-1][*bitStartIndex], 8-(*fatherY%8))
	}
	*nowBitM = clearRightBits(bitMap[*fatherX][*bitStartIndex], 8-(*fatherY%8))
	*PLastIsObstacle = false
	*NLastIsObstacle = false
	*mayFindJumpNode = false
	*mayEndPointLoopTimes = m.calHEndPointLoopTimes(temJumpNode, false)
	*loopTimes = 0
}

func (m *Map) reset_Down_TemData(temJumpNode *Node, fatherX, fatherY, bitStartIndex, mayEndPointLoopTimes, loopTimes *int, nowBitN, nowBitM, nowBitP *uint8, PLastIsObstacle, NLastIsObstacle, mayFindJumpNode *bool, availableN, availableP bool, bitMap [][]uint8) {
	*fatherX, *fatherY = temJumpNode.X, temJumpNode.Y
	*bitStartIndex = *fatherX / 8
	if availableP {
		*nowBitP = clearLeftBits(bitMap[*fatherY+1][*bitStartIndex], (*fatherX%8)+1)
	}
	if availableN {
		*nowBitN = clearLeftBits(bitMap[*fatherY-1][*bitStartIndex], (*fatherX%8)+1)
	}
	*nowBitM = clearLeftBits(bitMap[*fatherY][*bitStartIndex], (*fatherX%8)+1)
	*PLastIsObstacle = false
	*NLastIsObstacle = false
	*mayFindJumpNode = false
	*mayEndPointLoopTimes = m.calVEndPointLoopTimes(temJumpNode, true)
	*loopTimes = 0
}

func (m *Map) reset_Up_TemData(temJumpNode *Node, fatherX, fatherY, bitStartIndex, mayEndPointLoopTimes, loopTimes *int, nowBitN, nowBitM, nowBitP *uint8, PLastIsObstacle, NLastIsObstacle, mayFindJumpNode *bool, availableN, availableP bool, bitMap [][]uint8) {
	*fatherX, *fatherY = temJumpNode.X, temJumpNode.Y
	*bitStartIndex = *fatherX / 8
	if availableP {
		*nowBitP = clearRightBits(bitMap[*fatherY+1][*bitStartIndex], 8-(*fatherX%8))
	}
	if availableN {
		*nowBitN = clearRightBits(bitMap[*fatherY-1][*bitStartIndex], 8-(*fatherX%8))
	}
	*nowBitM = clearRightBits(bitMap[*fatherY][*bitStartIndex], 8-(*fatherX%8))
	*PLastIsObstacle = false
	*NLastIsObstacle = false
	*mayFindJumpNode = false
	*mayEndPointLoopTimes = m.calVEndPointLoopTimes(temJumpNode, false)
	*loopTimes = 0
}

// 注意特殊情况 : 8位图全为0时 --   8位图全为1时 --
// 1次方法调用即可获得跳点 从水平方向获取当前8位可能的跳点 delta是当前方向正负 以下x+或右y+为正
func (m *Map) get_Right_HorizontalRealJumpPoint(availableP, availableN, fatherIsDiagonal bool, fatherNode *Node) (*Node, bool) {
	//定义位图 记录位图遍历的次数 记录当前bit --
	delta := 1
	fatherX, fatherY := fatherNode.X, fatherNode.Y
	var bitMap [][]uint8
	var bitMapLen = 0
	var loopTimes, PJumpTimes, NJumpTimes = 0, 0, 0
	var nowBitP, nowBitM, nowBitN uint8 = 0, 0, 0
	var bitStartIndex = fatherY / 8
	var MZeroTimes = -1
	var PLastIsObstacle, NLastIsObstacle = false, false
	var mayEndPointLoopTimes = m.calHEndPointLoopTimes(fatherNode, true)
	var temJumpNode *Node = nil
	var temJumpNewDir uint8 = 0
	var mayFindJumpNode = false
	//============================================Loop
	//0.选择位图
	bitMap = m.LeftToRightBitMap
	bitMapLen = len(bitMap[fatherX])
	//1.循环获取可能的跳点
	if availableP && availableN {
		//2.第1次先清除需要的当前8位部分左边所有位 置为0
		nowBitN = clearLeftBits(bitMap[fatherX-1][bitStartIndex], (fatherY%8)+1)
		nowBitM = clearLeftBits(bitMap[fatherX][bitStartIndex], (fatherY%8)+1)
		nowBitP = clearLeftBits(bitMap[fatherX+1][bitStartIndex], (fatherY%8)+1)
		//--------------------------------------------------------PMN主循环
		for {
			//有障碍物连续性的上1位 位图底必然是障碍 当前位不是障碍物时,即视为找到跳点的下一个 -- 就是找前导1的数量 特殊：全部都是障碍 前导1数量为8
			//无障碍物连续性的上1位 位图必然全是非障碍 正常位运算自己
			//P
			if PLastIsObstacle {
				PJumpTimes = calLeadingOne_8bit(nowBitP)
			} else {
				PJumpTimes = calLeadingZero_8bit((nowBitP >> 1) & ^nowBitP)
			}
			//N
			if NLastIsObstacle {
				NJumpTimes = calLeadingOne_8bit(nowBitN)
			} else {
				NJumpTimes = calLeadingZero_8bit((nowBitN >> 1) & ^nowBitN)
			}
			//M 只需要看障碍物位置
			MZeroTimes = calLeadingZero_8bit(nowBitM)
			//---------------------------------------跳点分情况判断
			if mayEndPointLoopTimes == loopTimes { //0.(单独加1个)如果当前处于终点位图 那么回退原始方法 不管结果如何 都要结束 因为终点位图永远都是最后一次遍历
				if mayEndPointLoopTimes == 0 { //起始位图就是终点位图 起始点是fatherNode的下1个点 当前方向Right
					return m.doHMayEndPointBitJumpNode(fatherX, fatherY+1, fatherNode, 1, availableP, availableN, fatherIsDiagonal)
				} else { //终点位图不是起始位图 那么至少有2个及以上位图 起始点为终点位图起始位置的前1个 也就是上1个位图的末尾--所以有可能与父节点重合 当前方向Right
					return m.doHMayEndPointBitJumpNode(fatherX, (((fatherY/8)+loopTimes)*8)-1, fatherNode, 1, availableP, availableN, fatherIsDiagonal)
				}
			}
			if PJumpTimes < NJumpTimes { //P小
				//-------------------------------------1.P跳点有效
				if MZeroTimes >= PJumpTimes+1 {
					temJumpNode = m.Nodes[fatherX][(((fatherY/8)+loopTimes)*8)+PJumpTimes-1]
					temJumpNewDir = Bit1 | Bit5
					mayFindJumpNode = true
				} else {
					return nil, false //本次逻辑没有跳点 退出逻辑
				}
			} else if PJumpTimes > NJumpTimes { //N小
				//-------------------------------------2.N跳点有效
				if MZeroTimes >= NJumpTimes+1 {
					temJumpNode = m.Nodes[fatherX][(((fatherY/8)+loopTimes)*8)+NJumpTimes-1]
					temJumpNewDir = Bit1 | Bit4
					mayFindJumpNode = true
				} else {
					return nil, false //本次逻辑没有跳点 退出逻辑
				}
			} else { //PN相等
				//-------------------------------------3.PN次数相同
				if PJumpTimes == 8 { //PN都无跳点
					if MZeroTimes != 8 {
						return nil, false //自己行碰到障碍 后续跳点无效
					}
				} else {
					if MZeroTimes >= PJumpTimes+1 { //PN跳点都有效
						temJumpNode = m.Nodes[fatherX][(((fatherY/8)+loopTimes)*8)+PJumpTimes-1]
						temJumpNewDir = Bit1 | Bit4 | Bit5
						mayFindJumpNode = true
					} else {
						return nil, false //本次逻辑没有跳点 退出逻辑
					}
				}
			}
			//如果之前的操作找到了可能性跳点或强制邻居
			if mayFindJumpNode {
				if fatherIsDiagonal { //父节点是斜向点拓展探索 父节点必定不在open或close中
					if temJumpNode.isInOpenList1() || temJumpNode.isInCloseList1() { //当前强制邻居在open中 父节点不在open或close中 不需要回溯,当前强制在close中也跳过 并且跳过 --位运算要以当前强制邻居为虚拟父节点继续遍历
						m.reset_Right_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else { //否则强制邻居生效 但是跳点是斜向点本身 不是强制邻居 返回nil,找到有效强制邻居true
						return nil, true
					}
				} else { //父节点是第一级源点 那么父节点必定处于close中
					if temJumpNode.isInOpenList1() { //跳点处于open 回溯当前跳点 跳过
						m.fixRayCastNode_InOpen_New(temJumpNode, fatherNode)                                                                                                                                                                          //回溯
						m.reset_Right_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else if temJumpNode.isInCloseList1() { //跳点处于close中 不回溯 跳过
						m.reset_Right_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else { //跳点不处于open和close 跳点有效 加入open
						return m.addJumpNode_ToOpen_New(fatherNode, temJumpNode, temJumpNewDir) //添加open 新方向
					}
				}
			}
			//---------------------------------------PN障碍连续性判断 如果是第1次最后1位开始 那么不应该判断障碍物连续性 因为这次位图都是无效的
			if loopTimes != 0 || fatherY%8 != 7 {
				PLastIsObstacle = 1&nowBitP != 0
				NLastIsObstacle = 1&nowBitN != 0
			}
			//----------------------------------------更新位图loopTimes++ bitStartIndex += delta 注意位图越界
			loopTimes++
			bitStartIndex += delta
			if bitStartIndex >= bitMapLen { //本次位图已经遍历完没找到跳点
				return nil, false
			}
			nowBitP = bitMap[fatherX+1][bitStartIndex]
			nowBitM = bitMap[fatherX][bitStartIndex]
			nowBitN = bitMap[fatherX-1][bitStartIndex]
		}
	} else if availableP {
		//2.第1次先清除需要的当前8位部分左边所有位 置为0
		nowBitM = clearLeftBits(bitMap[fatherX][bitStartIndex], (fatherY%8)+1)
		nowBitP = clearLeftBits(bitMap[fatherX+1][bitStartIndex], (fatherY%8)+1)
		//--------------------------------------------------------PM主循环
		for {
			//有障碍物连续性的上1位 位图底必然是障碍 当前位不是障碍物时,即视为找到跳点的下一个 -- 就是找前导1的数量 特殊：全部都是障碍 前导1数量为8
			//无障碍物连续性的上1位 位图必然全是非障碍 正常位运算自己
			//P
			if PLastIsObstacle {
				PJumpTimes = calLeadingOne_8bit(nowBitP)
			} else {
				PJumpTimes = calLeadingZero_8bit((nowBitP >> 1) & ^nowBitP)
			}
			//M 只需要看障碍物位置
			MZeroTimes = calLeadingZero_8bit(nowBitM)
			//---------------------------------------跳点分情况判断
			if mayEndPointLoopTimes == loopTimes { //0.(单独加1个)如果当前处于终点位图 那么回退原始方法 不管结果如何 都要结束 因为终点位图永远都是最后一次遍历
				if mayEndPointLoopTimes == 0 { //起始位图就是终点位图 起始点是fatherNode的下1个点 当前方向Right
					return m.doHMayEndPointBitJumpNode(fatherX, fatherY+1, fatherNode, 1, availableP, availableN, fatherIsDiagonal)
				} else { //终点位图不是起始位图 那么至少有2个及以上位图 起始点为终点位图起始位置的前1个 也就是上1个位图的末尾--所以有可能与父节点重合 当前方向Right
					return m.doHMayEndPointBitJumpNode(fatherX, (((fatherY/8)+loopTimes)*8)-1, fatherNode, 1, availableP, availableN, fatherIsDiagonal)
				}
			}
			if PJumpTimes == 8 { //P无跳点
				//---------------------------------------1.M已有障碍
				if MZeroTimes != 8 {
					return nil, false //自己行碰到障碍 后续跳点无效
				}
			} else {
				//---------------------------------------2.P跳点有效
				if MZeroTimes >= PJumpTimes+1 { //P跳点都有效
					temJumpNode = m.Nodes[fatherX][(((fatherY/8)+loopTimes)*8)+PJumpTimes-1]
					temJumpNewDir = Bit1 | Bit5
					mayFindJumpNode = true
				} else {
					return nil, false //本次逻辑没有跳点 退出逻辑
				}
			}
			//看是否有可能性跳点
			if mayFindJumpNode {
				if fatherIsDiagonal { //父节点是斜向点拓展探索 父节点必定不在open或close中
					if temJumpNode.isInOpenList1() || temJumpNode.isInCloseList1() { //当前强制邻居在open中 父节点不在open或close中 不需要回溯,当前强制在close中也跳过 并且跳过 --位运算要以当前强制邻居为虚拟父节点继续遍历
						m.reset_Right_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else { //否则强制邻居生效 但是跳点是斜向点本身 不是强制邻居 返回nil,找到有效强制邻居true
						return nil, true
					}
				} else { //父节点是第一级源点 那么父节点必定处于close中
					if temJumpNode.isInOpenList1() { //跳点处于open 回溯当前跳点 跳过
						m.fixRayCastNode_InOpen_New(temJumpNode, fatherNode)                                                                                                                                                                          //回溯
						m.reset_Right_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else if temJumpNode.isInCloseList1() { //跳点处于close中 不回溯 跳过
						m.reset_Right_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else { //跳点不处于open和close 跳点有效 加入open
						return m.addJumpNode_ToOpen_New(fatherNode, temJumpNode, temJumpNewDir) //添加open 新方向
					}
				}
			}
			//---------------------------------------PN障碍连续性判断 如果是第1次最后1位开始 那么不应该判断障碍物连续性 因为这次位图都是无效的
			if loopTimes != 0 || fatherY%8 != 7 {
				PLastIsObstacle = 1&nowBitP != 0
			}
			//----------------------------------------更新位图loopTimes++ bitStartIndex += delta 注意位图越界
			loopTimes++
			bitStartIndex += delta
			if bitStartIndex >= bitMapLen { //本次位图已经遍历完没找到跳点
				return nil, false
			}
			nowBitP = bitMap[fatherX+1][bitStartIndex]
			nowBitM = bitMap[fatherX][bitStartIndex]
		}
	} else if availableN {
		//2.第1次先清除需要的当前8位部分左边所有位 置为0
		nowBitN = clearLeftBits(bitMap[fatherX-1][bitStartIndex], (fatherY%8)+1)
		nowBitM = clearLeftBits(bitMap[fatherX][bitStartIndex], (fatherY%8)+1)
		//--------------------------------------------------------MN主循环
		for {
			//有障碍物连续性的上1位 位图底必然是障碍 当前位不是障碍物时,即视为找到跳点的下一个 -- 就是找前导1的数量 特殊：全部都是障碍 前导1数量为8
			//无障碍物连续性的上1位 位图必然全是非障碍 正常位运算自己
			//N
			if NLastIsObstacle {
				NJumpTimes = calLeadingOne_8bit(nowBitN)
			} else {
				NJumpTimes = calLeadingZero_8bit((nowBitN >> 1) & ^nowBitN)
			}
			//M 只需要看障碍物位置
			MZeroTimes = calLeadingZero_8bit(nowBitM)
			//---------------------------------------跳点分情况判断
			if mayEndPointLoopTimes == loopTimes { //0.(单独加1个)如果当前处于终点位图 那么回退原始方法 不管结果如何 都要结束 因为终点位图永远都是最后一次遍历
				if mayEndPointLoopTimes == 0 { //起始位图就是终点位图 起始点是fatherNode的下1个点 当前方向Right
					return m.doHMayEndPointBitJumpNode(fatherX, fatherY+1, fatherNode, 1, availableP, availableN, fatherIsDiagonal)
				} else { //终点位图不是起始位图 那么至少有2个及以上位图 起始点为终点位图起始位置的前1个 也就是上1个位图的末尾--所以有可能与父节点重合 当前方向Right
					return m.doHMayEndPointBitJumpNode(fatherX, (((fatherY/8)+loopTimes)*8)-1, fatherNode, 1, availableP, availableN, fatherIsDiagonal)
				}
			}
			if NJumpTimes == 8 { //N无跳点
				//---------------------------------------1.M已有障碍
				if MZeroTimes != 8 {
					return nil, false //自己行碰到障碍 后续跳点无效
				}
			} else {
				//---------------------------------------2.N跳点有效
				if MZeroTimes >= NJumpTimes+1 { //N跳点都有效
					temJumpNode = m.Nodes[fatherX][(((fatherY/8)+loopTimes)*8)+NJumpTimes-1]
					temJumpNewDir = Bit1 | Bit4
					mayFindJumpNode = true
				} else {
					return nil, false //本次逻辑没有跳点 退出逻辑
				}
			}
			//看是否有可能性跳点
			if mayFindJumpNode {
				if fatherIsDiagonal { //父节点是斜向点拓展探索 父节点必定不在open或close中
					if temJumpNode.isInOpenList1() || temJumpNode.isInCloseList1() { //当前强制邻居在open中 父节点不在open或close中 不需要回溯,当前强制在close中也跳过 并且跳过 --位运算要以当前强制邻居为虚拟父节点继续遍历
						m.reset_Right_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else { //否则强制邻居生效 但是跳点是斜向点本身 不是强制邻居 返回nil,找到有效强制邻居true
						return nil, true
					}
				} else { //父节点是第一级源点 那么父节点必定处于close中
					if temJumpNode.isInOpenList1() { //跳点处于open 回溯当前跳点 跳过
						m.fixRayCastNode_InOpen_New(temJumpNode, fatherNode)                                                                                                                                                                          //回溯
						m.reset_Right_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else if temJumpNode.isInCloseList1() { //跳点处于close中 不回溯 跳过
						m.reset_Right_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else { //跳点不处于open和close 跳点有效 加入open
						return m.addJumpNode_ToOpen_New(fatherNode, temJumpNode, temJumpNewDir) //添加open 新方向
					}
				}
			}
			//---------------------------------------PN障碍连续性判断 如果是第1次最后1位开始 那么不应该判断障碍物连续性 因为这次位图都是无效的
			if loopTimes != 0 || fatherY%8 != 7 {
				NLastIsObstacle = 1&nowBitN != 0
			}
			//----------------------------------------更新位图loopTimes++ bitStartIndex += delta 注意位图越界
			loopTimes++
			bitStartIndex += delta
			if bitStartIndex >= bitMapLen { //本次位图已经遍历完没找到跳点
				return nil, false
			}
			nowBitM = bitMap[fatherX][bitStartIndex]
			nowBitN = bitMap[fatherX-1][bitStartIndex]
		}
	}
	//默认未找到跳点
	return nil, false
}

func (m *Map) get_Left_HorizontalRealJumpPoint(availableP, availableN, fatherIsDiagonal bool, fatherNode *Node) (*Node, bool) {
	//定义位图 记录位图遍历的次数 记录当前bit --
	delta := -1
	fatherX, fatherY := fatherNode.X, fatherNode.Y
	var bitMap [][]uint8
	var loopTimes, PJumpTimes, NJumpTimes = 0, 0, 0
	var nowBitP, nowBitM, nowBitN uint8 = 0, 0, 0
	var bitStartIndex = fatherY / 8
	var MZeroTimes = -1
	var PLastIsObstacle, NLastIsObstacle = false, false
	var mayEndPointLoopTimes = m.calHEndPointLoopTimes(fatherNode, false)
	var temJumpNode *Node = nil
	var temJumpNewDir uint8 = 0
	var mayFindJumpNode = false
	//===============================Loop
	bitMap = m.LeftToRightBitMap
	//1.循环获取可能的跳点
	if availableP && availableN {
		//2.第1次先清除需要的当前8位部分左边所有位 置为0
		nowBitN = clearRightBits(bitMap[fatherX-1][bitStartIndex], 8-(fatherY%8))
		nowBitM = clearRightBits(bitMap[fatherX][bitStartIndex], 8-(fatherY%8))
		nowBitP = clearRightBits(bitMap[fatherX+1][bitStartIndex], 8-(fatherY%8))
		//--------------------------------------------------------PMN主循环
		for {
			//有障碍物连续性的上1位 位图底必然是障碍 当前位不是障碍物时,即视为找到跳点的下一个 -- 就是找后导1的数量 特殊：全部都是障碍 后导1数量为8
			//无障碍物连续性的上1位 位图必然全是非障碍 正常位运算自己
			//P
			if PLastIsObstacle {
				PJumpTimes = calTrailingOne_8bit(nowBitP)
			} else {
				PJumpTimes = calTrailingZero_8bit((nowBitP << 1) & ^nowBitP)
			}
			//N
			if NLastIsObstacle {
				NJumpTimes = calTrailingOne_8bit(nowBitN)
			} else {
				NJumpTimes = calTrailingZero_8bit((nowBitN << 1) & ^nowBitN)
			}
			//M 只需要看障碍物位置
			MZeroTimes = calTrailingZero_8bit(nowBitM)
			//---------------------------------------跳点分情况判断  没写完
			if mayEndPointLoopTimes == loopTimes { //0.(单独加1个)如果当前处于终点位图 那么回退原始方法 不管结果如何 都要结束 因为终点位图永远都是最后一次遍历
				if mayEndPointLoopTimes == 0 { //起始位图就是终点位图 起始点是fatherNode的下1个点 当前方向 Left
					return m.doHMayEndPointBitJumpNode(fatherX, fatherY-1, fatherNode, -1, availableP, availableN, fatherIsDiagonal)
				} else { //终点位图不是起始位图 起始点为当前位图末尾的下1个 也就是上1位图的第1个 当前方向Right
					return m.doHMayEndPointBitJumpNode(fatherX, (((fatherY/8)-loopTimes)*8)+7+1, fatherNode, -1, availableP, availableN, fatherIsDiagonal)
				}
			}
			if PJumpTimes < NJumpTimes { //P小
				//-------------------------------------1.P跳点有效
				if MZeroTimes >= PJumpTimes+1 {
					temJumpNode = m.Nodes[fatherX][(((fatherY/8)+1-loopTimes)*8)-PJumpTimes]
					temJumpNewDir = Bit3 | Bit6
					mayFindJumpNode = true
				} else {
					return nil, false //本次逻辑没有跳点 退出逻辑
				}
			} else if PJumpTimes > NJumpTimes { //N小
				//-------------------------------------2.N跳点有效
				if MZeroTimes >= NJumpTimes+1 {
					temJumpNode = m.Nodes[fatherX][(((fatherY/8)+1-loopTimes)*8)-NJumpTimes]
					temJumpNewDir = Bit3 | Bit7
					mayFindJumpNode = true
				} else {
					return nil, false //本次逻辑没有跳点 退出逻辑
				}
			} else { //PN相等
				//-------------------------------------3.PN次数相同
				if PJumpTimes == 8 { //PN都无跳点
					if MZeroTimes != 8 {
						return nil, false //本次逻辑没有跳点 退出逻辑
					}
				} else {
					if MZeroTimes >= PJumpTimes+1 { //PN跳点都有效
						temJumpNode = m.Nodes[fatherX][(((fatherY/8)+1-loopTimes)*8)-NJumpTimes]
						temJumpNewDir = Bit3 | Bit7 | Bit6
						mayFindJumpNode = true
					} else {
						return nil, false //本次逻辑没有跳点 退出逻辑
					}
				}
			}
			//如果之前的操作找到了可能性跳点或强制邻居
			if mayFindJumpNode {
				if fatherIsDiagonal { //父节点是斜向点拓展探索 父节点必定不在open或close中
					if temJumpNode.isInOpenList1() || temJumpNode.isInCloseList1() { //当前强制邻居在open中 父节点不在open或close中 不需要回溯,当前强制在close中也跳过 并且跳过 --位运算要以当前强制邻居为虚拟父节点继续遍历
						m.reset_Left_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else { //否则强制邻居生效 但是跳点是斜向点本身 不是强制邻居 返回nil,找到有效强制邻居true
						return nil, true
					}
				} else { //父节点是第一级源点 那么父节点必定处于close中
					if temJumpNode.isInOpenList1() { //跳点处于open 回溯当前跳点 跳过
						m.fixRayCastNode_InOpen_New(temJumpNode, fatherNode)                                                                                                                                                                         //回溯
						m.reset_Left_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else if temJumpNode.isInCloseList1() { //跳点处于close中 不回溯 跳过
						m.reset_Left_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else { //跳点不处于open和close 跳点有效 加入open
						return m.addJumpNode_ToOpen_New(fatherNode, temJumpNode, temJumpNewDir) //添加open 新方向
					}
				}
			}
			//---------------------------------------PN障碍连续性判断 如果父节点是第1次最后1位开始(反向除8余0是最后1位) 那么不应该判断障碍物连续性 因为这次位图都是无效的
			if loopTimes != 0 || fatherY%8 != 0 {
				PLastIsObstacle = 128&nowBitP != 0
				NLastIsObstacle = 128&nowBitN != 0
			}
			//----------------------------------------更新位图loopTimes++ bitStartIndex += delta 注意位图越界
			loopTimes++
			bitStartIndex += delta
			if bitStartIndex < 0 { //本次位图已经遍历完没找到跳点
				return nil, false
			}
			nowBitP = bitMap[fatherX+1][bitStartIndex]
			nowBitM = bitMap[fatherX][bitStartIndex]
			nowBitN = bitMap[fatherX-1][bitStartIndex]
		}
	} else if availableP {
		//2.第1次先清除需要的当前8位部分左边所有位 置为0
		nowBitM = clearRightBits(bitMap[fatherX][bitStartIndex], 8-(fatherY%8))
		nowBitP = clearRightBits(bitMap[fatherX+1][bitStartIndex], 8-(fatherY%8))
		//--------------------------------------------------------PM主循环
		for {
			//有障碍物连续性的上1位 位图底必然是障碍 当前位不是障碍物时,即视为找到跳点的下一个 -- 就是找后导1的数量 特殊：全部都是障碍 后导1数量为8
			//无障碍物连续性的上1位 位图必然全是非障碍 正常位运算自己
			//P
			if PLastIsObstacle {
				PJumpTimes = calTrailingOne_8bit(nowBitP)
			} else {
				PJumpTimes = calTrailingZero_8bit((nowBitP << 1) & ^nowBitP)
			}
			//M 只需要看障碍物位置
			MZeroTimes = calTrailingZero_8bit(nowBitM)
			//---------------------------------------跳点分情况判断
			if mayEndPointLoopTimes == loopTimes { //0.(单独加1个)如果当前处于终点位图 那么回退原始方法 不管结果如何 都要结束 因为终点位图永远都是最后一次遍历
				if mayEndPointLoopTimes == 0 { //起始位图就是终点位图 起始点是fatherNode的下1个点 当前方向 Left
					return m.doHMayEndPointBitJumpNode(fatherX, fatherY-1, fatherNode, -1, availableP, availableN, fatherIsDiagonal)
				} else { //终点位图不是起始位图 起始点为当前位图末尾的下1个 也就是上1位图的第1个 当前方向Right
					return m.doHMayEndPointBitJumpNode(fatherX, (((fatherY/8)-loopTimes)*8)+7+1, fatherNode, -1, availableP, availableN, fatherIsDiagonal)
				}
			}
			if PJumpTimes == 8 { //P无跳点
				//---------------------------------------1.M已有障碍
				if MZeroTimes != 8 {
					return nil, false //本次逻辑没有跳点 退出逻辑
				}
			} else {
				//---------------------------------------2.P跳点有效
				if MZeroTimes >= PJumpTimes+1 { //P跳点都有效
					temJumpNode = m.Nodes[fatherX][(((fatherY/8)+1-loopTimes)*8)-PJumpTimes]
					temJumpNewDir = Bit3 | Bit6
					mayFindJumpNode = true
				} else {
					return nil, false //本次逻辑没有跳点 退出逻辑
				}
			}
			//如果之前的操作找到了可能性跳点或强制邻居
			if mayFindJumpNode {
				if fatherIsDiagonal { //父节点是斜向点拓展探索 父节点必定不在open或close中
					if temJumpNode.isInOpenList1() || temJumpNode.isInCloseList1() { //当前强制邻居在open中 父节点不在open或close中 不需要回溯,当前强制在close中也跳过 并且跳过 --位运算要以当前强制邻居为虚拟父节点继续遍历
						m.reset_Left_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else { //否则强制邻居生效 但是跳点是斜向点本身 不是强制邻居 返回nil,找到有效强制邻居true
						return nil, true
					}
				} else { //父节点是第一级源点 那么父节点必定处于close中
					if temJumpNode.isInOpenList1() { //跳点处于open 回溯当前跳点 跳过
						m.fixRayCastNode_InOpen_New(temJumpNode, fatherNode)                                                                                                                                                                         //回溯
						m.reset_Left_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else if temJumpNode.isInCloseList1() { //跳点处于close中 不回溯 跳过
						m.reset_Left_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else { //跳点不处于open和close 跳点有效 加入open
						return m.addJumpNode_ToOpen_New(fatherNode, temJumpNode, temJumpNewDir) //添加open 新方向
					}
				}
			}
			//---------------------------------------PN障碍连续性判断 如果父节点是第1次最后1位开始(反向除8余0是最后1位) 那么不应该判断障碍物连续性 因为这次位图都是无效的
			if loopTimes != 0 || fatherY%8 != 0 {
				PLastIsObstacle = 128&nowBitP != 0
			}
			//----------------------------------------更新位图loopTimes++ bitStartIndex += delta 注意位图越界
			loopTimes++
			bitStartIndex += delta
			if bitStartIndex < 0 { //本次位图已经遍历完没找到跳点
				return nil, false
			}
			nowBitP = bitMap[fatherX+1][bitStartIndex]
			nowBitM = bitMap[fatherX][bitStartIndex]
		}
	} else if availableN {
		//2.第1次先清除需要的当前8位部分左边所有位 置为0
		nowBitN = clearRightBits(bitMap[fatherX-1][bitStartIndex], 8-(fatherY%8))
		nowBitM = clearRightBits(bitMap[fatherX][bitStartIndex], 8-(fatherY%8))
		//--------------------------------------------------------MN主循环
		for {
			//有障碍物连续性的上1位 位图底必然是障碍 当前位不是障碍物时,即视为找到跳点的下一个 -- 就是找后导1的数量 特殊：全部都是障碍 后导1数量为8
			//无障碍物连续性的上1位 位图必然全是非障碍 正常位运算自己
			//N
			if NLastIsObstacle {
				NJumpTimes = calTrailingOne_8bit(nowBitN)
			} else {
				NJumpTimes = calTrailingZero_8bit((nowBitN << 1) & ^nowBitN)
			}
			//M 只需要看障碍物位置
			MZeroTimes = calTrailingZero_8bit(nowBitM)
			//---------------------------------------跳点分情况判断
			if mayEndPointLoopTimes == loopTimes { //0.(单独加1个)如果当前处于终点位图 那么回退原始方法 不管结果如何 都要结束 因为终点位图永远都是最后一次遍历
				if mayEndPointLoopTimes == 0 { //起始位图就是终点位图 起始点是fatherNode的下1个点 当前方向 Left
					return m.doHMayEndPointBitJumpNode(fatherX, fatherY-1, fatherNode, -1, availableP, availableN, fatherIsDiagonal)
				} else { //终点位图不是起始位图 起始点为当前位图末尾的下1个 也就是上1位图的第1个 当前方向Right
					return m.doHMayEndPointBitJumpNode(fatherX, (((fatherY/8)-loopTimes)*8)+7+1, fatherNode, -1, availableP, availableN, fatherIsDiagonal)
				}
			}
			if NJumpTimes == 8 { //N无跳点
				//---------------------------------------1.M已有障碍
				if MZeroTimes != 8 {
					return nil, false //本次逻辑没有跳点 退出逻辑
				}
			} else {
				//---------------------------------------2.N跳点有效
				if MZeroTimes >= NJumpTimes+1 { //N跳点都有效
					temJumpNode = m.Nodes[fatherX][(((fatherY/8)+1-loopTimes)*8)-NJumpTimes]
					temJumpNewDir = Bit3 | Bit7
					mayFindJumpNode = true
				} else {
					return nil, false //本次逻辑没有跳点 退出逻辑
				}
			}
			//如果之前的操作找到了可能性跳点或强制邻居
			if mayFindJumpNode {
				if fatherIsDiagonal { //父节点是斜向点拓展探索 父节点必定不在open或close中
					if temJumpNode.isInOpenList1() || temJumpNode.isInCloseList1() { //当前强制邻居在open中 父节点不在open或close中 不需要回溯,当前强制在close中也跳过 并且跳过 --位运算要以当前强制邻居为虚拟父节点继续遍历
						m.reset_Left_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else { //否则强制邻居生效 但是跳点是斜向点本身 不是强制邻居 返回nil,找到有效强制邻居true
						return nil, true
					}
				} else { //父节点是第一级源点 那么父节点必定处于close中
					if temJumpNode.isInOpenList1() { //跳点处于open 回溯当前跳点 跳过
						m.fixRayCastNode_InOpen_New(temJumpNode, fatherNode)                                                                                                                                                                         //回溯
						m.reset_Left_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else if temJumpNode.isInCloseList1() { //跳点处于close中 不回溯 跳过
						m.reset_Left_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else { //跳点不处于open和close 跳点有效 加入open
						return m.addJumpNode_ToOpen_New(fatherNode, temJumpNode, temJumpNewDir) //添加open 新方向
					}
				}
			}
			//---------------------------------------PN障碍连续性判断 如果父节点是第1次最后1位开始(反向除8余0是最后1位) 那么不应该判断障碍物连续性 因为这次位图都是无效的
			if loopTimes != 0 || fatherY%8 != 0 {
				NLastIsObstacle = 128&nowBitN != 0
			}
			//----------------------------------------更新位图loopTimes++ bitStartIndex += delta 注意位图越界
			loopTimes++
			bitStartIndex += delta
			if bitStartIndex < 0 { //本次位图已经遍历完没找到跳点
				return nil, false
			}
			nowBitM = bitMap[fatherX][bitStartIndex]
			nowBitN = bitMap[fatherX-1][bitStartIndex]
		}
	}
	//默认未找到跳点
	return nil, false
}

// 1次方法调用即可获得跳点 从垂直方向获取当前8位可能的跳点 index是预存的位图索引 delta是当前方向正负 以下x+或右y+为正
func (m *Map) get_Down_VerticalRealJumpPoint(availableP, availableN, fatherIsDiagonal bool, fatherNode *Node) (*Node, bool) {
	//定义位图 记录位图遍历的次数 记录当前bit --
	delta := 1
	fatherX, fatherY := fatherNode.X, fatherNode.Y
	var bitMap [][]uint8
	var bitMapLen = 0
	var loopTimes, PJumpTimes, NJumpTimes = 0, 0, 0
	var nowBitP, nowBitM, nowBitN uint8 = 0, 0, 0
	var bitStartIndex = fatherX / 8
	var MZeroTimes = -1
	var PLastIsObstacle, NLastIsObstacle = false, false
	var mayEndPointLoopTimes = m.calVEndPointLoopTimes(fatherNode, true)
	var temJumpNode *Node = nil
	var temJumpNewDir uint8 = 0
	var mayFindJumpNode = false
	//==================================================Loop
	//0.选择位图
	bitMap = m.UpToDownColsBitMap
	bitMapLen = len(bitMap[fatherY])
	//1.循环获取可能的跳点
	if availableP && availableN {
		//2.第1次先清除需要的当前8位部分左边所有位 置为0
		nowBitP = clearLeftBits(bitMap[fatherY+1][bitStartIndex], (fatherX%8)+1)
		nowBitM = clearLeftBits(bitMap[fatherY][bitStartIndex], (fatherX%8)+1)
		nowBitN = clearLeftBits(bitMap[fatherY-1][bitStartIndex], (fatherX%8)+1)
		//--------------------------------------------------------PMN主循环
		for {
			//有障碍物连续性的上1位 位图底必然是障碍 当前位不是障碍物时,即视为找到跳点的下一个 -- 就是找前导1的数量 特殊：全部都是障碍 前导1数量为8
			//无障碍物连续性的上1位 位图必然全是非障碍 正常位运算自己
			//P
			if PLastIsObstacle {
				PJumpTimes = calLeadingOne_8bit(nowBitP)
			} else {
				PJumpTimes = calLeadingZero_8bit((nowBitP >> 1) & ^nowBitP)
			}
			//N
			if NLastIsObstacle {
				NJumpTimes = calLeadingOne_8bit(nowBitN)
			} else {
				NJumpTimes = calLeadingZero_8bit((nowBitN >> 1) & ^nowBitN)
			}
			//M 只需要看障碍物位置
			MZeroTimes = calLeadingZero_8bit(nowBitM)
			//---------------------------------------跳点分情况判断
			if mayEndPointLoopTimes == loopTimes { //0.(单独加1个)如果当前处于终点位图 那么回退原始方法 不管结果如何 都要结束 因为终点位图永远都是最后一次遍历
				if mayEndPointLoopTimes == 0 { //起始位图就是终点位图 起始点是fatherNode的下1个点 当前方向 Left
					return m.doVMayEndPointBitJumpNode(fatherX+1, fatherY, fatherNode, 1, availableP, availableN, fatherIsDiagonal)
				} else { //终点位图不是起始位图 起始点为当前位图第1个的上1个 也就是上1个位图的末尾 当前方向Right
					return m.doVMayEndPointBitJumpNode((((fatherX/8)+loopTimes)*8)-1, fatherY, fatherNode, 1, availableP, availableN, fatherIsDiagonal)
				}
			}
			if PJumpTimes < NJumpTimes { //P小
				//-------------------------------------1.P跳点有效
				if MZeroTimes >= PJumpTimes+1 {
					temJumpNode = m.Nodes[(((fatherX/8)+loopTimes)*8)+PJumpTimes-1][fatherY]
					temJumpNewDir = Bit2 | Bit5
					mayFindJumpNode = true
				} else {
					return nil, false //本次逻辑没有跳点 退出逻辑
				}
			} else if PJumpTimes > NJumpTimes { //N小
				//-------------------------------------2.N跳点有效
				if MZeroTimes >= NJumpTimes+1 {
					temJumpNode = m.Nodes[(((fatherX/8)+loopTimes)*8)+NJumpTimes-1][fatherY]
					temJumpNewDir = Bit2 | Bit6
					mayFindJumpNode = true
				} else {
					return nil, false //本次逻辑没有跳点 退出逻辑
				}
			} else { //PN相等
				//-------------------------------------3.PN次数相同
				if PJumpTimes == 8 { //PN都无跳点
					if MZeroTimes != 8 {
						return nil, false //自己行碰到障碍 后续跳点无效
					}
				} else {
					if MZeroTimes >= PJumpTimes+1 { //PN跳点都有效
						temJumpNode = m.Nodes[(((fatherX/8)+loopTimes)*8)+PJumpTimes-1][fatherY]
						temJumpNewDir = Bit2 | Bit5 | Bit6
						mayFindJumpNode = true
					} else {
						return nil, false //本次逻辑没有跳点 退出逻辑
					}
				}
			}
			//如果之前的操作找到了可能性跳点或强制邻居
			if mayFindJumpNode {
				if fatherIsDiagonal { //父节点是斜向点拓展探索 父节点必定不在open或close中
					if temJumpNode.isInOpenList1() || temJumpNode.isInCloseList1() { //当前强制邻居在open中 父节点不在open或close中 不需要回溯,当前强制在close中也跳过 并且跳过 --位运算要以当前强制邻居为虚拟父节点继续遍历
						m.reset_Down_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else { //否则强制邻居生效 但是跳点是斜向点本身 不是强制邻居 返回nil,找到有效强制邻居true
						return nil, true
					}
				} else { //父节点是第一级源点 那么父节点必定处于close中
					if temJumpNode.isInOpenList1() { //跳点处于open 回溯当前跳点 跳过
						m.fixRayCastNode_InOpen_New(temJumpNode, fatherNode)                                                                                                                                                                         //回溯
						m.reset_Down_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else if temJumpNode.isInCloseList1() { //跳点处于close中 不回溯 跳过
						m.reset_Down_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else { //跳点不处于open和close 跳点有效 加入open
						return m.addJumpNode_ToOpen_New(fatherNode, temJumpNode, temJumpNewDir) //添加open 新方向
					}
				}
			}
			//---------------------------------------PN障碍连续性判断 如果是第1次最后1位开始 那么不应该判断障碍物连续性 因为这次位图都是无效的
			if loopTimes != 0 || fatherY%8 != 7 {
				PLastIsObstacle = 1&nowBitP != 0
				NLastIsObstacle = 1&nowBitN != 0
			}
			//----------------------------------------更新位图loopTimes++ bitStartIndex += delta 注意位图越界
			loopTimes++
			bitStartIndex += delta
			if bitStartIndex >= bitMapLen { //本次位图已经遍历完没找到跳点
				return nil, false
			}
			nowBitP = bitMap[fatherY+1][bitStartIndex]
			nowBitM = bitMap[fatherY][bitStartIndex]
			nowBitN = bitMap[fatherY-1][bitStartIndex]
		}
	} else if availableP {
		//2.第1次先清除需要的当前8位部分左边所有位 置为0
		nowBitP = clearLeftBits(bitMap[fatherY+1][bitStartIndex], (fatherX%8)+1)
		nowBitM = clearLeftBits(bitMap[fatherY][bitStartIndex], (fatherX%8)+1)
		//--------------------------------------------------------PM主循环
		for {
			//有障碍物连续性的上1位 位图底必然是障碍 当前位不是障碍物时,即视为找到跳点的下一个 -- 就是找前导1的数量 特殊：全部都是障碍 前导1数量为8
			//无障碍物连续性的上1位 位图必然全是非障碍 正常位运算自己
			//P
			if PLastIsObstacle {
				PJumpTimes = calLeadingOne_8bit(nowBitP)
			} else {
				PJumpTimes = calLeadingZero_8bit((nowBitP >> 1) & ^nowBitP)
			}
			//M 只需要看障碍物位置
			MZeroTimes = calLeadingZero_8bit(nowBitM)
			//---------------------------------------跳点分情况判断
			if mayEndPointLoopTimes == loopTimes { //0.(单独加1个)如果当前处于终点位图 那么回退原始方法 不管结果如何 都要结束 因为终点位图永远都是最后一次遍历
				if mayEndPointLoopTimes == 0 { //起始位图就是终点位图 起始点是fatherNode的下1个点 当前方向 Left
					return m.doVMayEndPointBitJumpNode(fatherX+1, fatherY, fatherNode, 1, availableP, availableN, fatherIsDiagonal)
				} else { //终点位图不是起始位图 起始点为当前位图第1个的上1个 也就是上1个位图的末尾 当前方向Right
					return m.doVMayEndPointBitJumpNode((((fatherX/8)+loopTimes)*8)-1, fatherY, fatherNode, 1, availableP, availableN, fatherIsDiagonal)
				}
			}
			if PJumpTimes == 8 { //P无跳点 或者跳点是最后1个 不过也只能在下一次位图知道
				//---------------------------------------1.M已有障碍
				if MZeroTimes != 8 {
					return nil, false //自己行碰到障碍 后续跳点无效
				}
			} else {
				//---------------------------------------2.P跳点有效
				if MZeroTimes >= PJumpTimes+1 { //P跳点都有效
					temJumpNode = m.Nodes[(((fatherX/8)+loopTimes)*8)+PJumpTimes-1][fatherY]
					temJumpNewDir = Bit2 | Bit5
					mayFindJumpNode = true
				} else {
					return nil, false //本次逻辑没有跳点 退出逻辑
				}
			}
			//如果之前的操作找到了可能性跳点或强制邻居
			if mayFindJumpNode {
				if fatherIsDiagonal { //父节点是斜向点拓展探索 父节点必定不在open或close中
					if temJumpNode.isInOpenList1() || temJumpNode.isInCloseList1() { //当前强制邻居在open中 父节点不在open或close中 不需要回溯,当前强制在close中也跳过 并且跳过 --位运算要以当前强制邻居为虚拟父节点继续遍历
						m.reset_Down_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else { //否则强制邻居生效 但是跳点是斜向点本身 不是强制邻居 返回nil,找到有效强制邻居true
						return nil, true
					}
				} else { //父节点是第一级源点 那么父节点必定处于close中
					if temJumpNode.isInOpenList1() { //跳点处于open 回溯当前跳点 跳过
						m.fixRayCastNode_InOpen_New(temJumpNode, fatherNode)                                                                                                                                                                         //回溯
						m.reset_Down_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else if temJumpNode.isInCloseList1() { //跳点处于close中 不回溯 跳过
						m.reset_Down_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else { //跳点不处于open和close 跳点有效 加入open
						return m.addJumpNode_ToOpen_New(fatherNode, temJumpNode, temJumpNewDir) //添加open 新方向
					}
				}
			}
			//---------------------------------------PN障碍连续性判断 如果是第1次最后1位开始 那么不应该判断障碍物连续性 因为这次位图都是无效的
			if loopTimes != 0 || fatherY%8 != 7 {
				PLastIsObstacle = 1&nowBitP != 0
			}
			//----------------------------------------更新位图loopTimes++ bitStartIndex += delta 注意位图越界
			loopTimes++
			bitStartIndex += delta
			if bitStartIndex >= bitMapLen { //本次位图已经遍历完没找到跳点
				return nil, false
			}
			nowBitP = bitMap[fatherY+1][bitStartIndex]
			nowBitM = bitMap[fatherY][bitStartIndex]
		}
	} else if availableN {
		//2.第1次先清除需要的当前8位部分左边所有位 置为0
		nowBitM = clearLeftBits(bitMap[fatherY][bitStartIndex], (fatherX%8)+1)
		nowBitN = clearLeftBits(bitMap[fatherY-1][bitStartIndex], (fatherX%8)+1)
		//--------------------------------------------------------MN主循环
		for {
			//有障碍物连续性的上1位 位图底必然是障碍 当前位不是障碍物时,即视为找到跳点的下一个 -- 就是找前导1的数量 特殊：全部都是障碍 前导1数量为8
			//无障碍物连续性的上1位 位图必然全是非障碍 正常位运算自己
			//N
			if NLastIsObstacle {
				NJumpTimes = calLeadingOne_8bit(nowBitN)
			} else {
				NJumpTimes = calLeadingZero_8bit((nowBitN >> 1) & ^nowBitN)
			}
			//M 只需要看障碍物位置
			MZeroTimes = calLeadingZero_8bit(nowBitM)
			//---------------------------------------跳点分情况判断
			if mayEndPointLoopTimes == loopTimes { //0.(单独加1个)如果当前处于终点位图 那么回退原始方法 不管结果如何 都要结束 因为终点位图永远都是最后一次遍历
				if mayEndPointLoopTimes == 0 { //起始位图就是终点位图 起始点是fatherNode的下1个点 当前方向 Left
					return m.doVMayEndPointBitJumpNode(fatherX+1, fatherY, fatherNode, 1, availableP, availableN, fatherIsDiagonal)
				} else { //终点位图不是起始位图 起始点为当前位图第1个的上1个 也就是上1个位图的末尾 当前方向Right
					return m.doVMayEndPointBitJumpNode((((fatherX/8)+loopTimes)*8)-1, fatherY, fatherNode, 1, availableP, availableN, fatherIsDiagonal)
				}
			}
			if NJumpTimes == 8 { //N无跳点
				//---------------------------------------1.M已有障碍
				if MZeroTimes != 8 {
					return nil, false //自己行碰到障碍 后续跳点无效
				}
			} else {
				//---------------------------------------2.N跳点有效
				if MZeroTimes >= NJumpTimes+1 { //N跳点都有效
					temJumpNode = m.Nodes[(((fatherX/8)+loopTimes)*8)+NJumpTimes-1][fatherY]
					temJumpNewDir = Bit2 | Bit6
					mayFindJumpNode = true
				} else {
					return nil, false //本次逻辑没有跳点 退出逻辑
				}
			}
			//如果之前的操作找到了可能性跳点或强制邻居
			if mayFindJumpNode {
				if fatherIsDiagonal { //父节点是斜向点拓展探索 父节点必定不在open或close中
					if temJumpNode.isInOpenList1() || temJumpNode.isInCloseList1() { //当前强制邻居在open中 父节点不在open或close中 不需要回溯,当前强制在close中也跳过 并且跳过 --位运算要以当前强制邻居为虚拟父节点继续遍历
						m.reset_Down_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else { //否则强制邻居生效 但是跳点是斜向点本身 不是强制邻居 返回nil,找到有效强制邻居true
						return nil, true
					}
				} else { //父节点是第一级源点 那么父节点必定处于close中
					if temJumpNode.isInOpenList1() { //跳点处于open 回溯当前跳点 跳过
						m.fixRayCastNode_InOpen_New(temJumpNode, fatherNode)                                                                                                                                                                         //回溯
						m.reset_Down_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else if temJumpNode.isInCloseList1() { //跳点处于close中 不回溯 跳过
						m.reset_Down_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else { //跳点不处于open和close 跳点有效 加入open
						return m.addJumpNode_ToOpen_New(fatherNode, temJumpNode, temJumpNewDir) //添加open 新方向
					}
				}
			}
			//---------------------------------------PN障碍连续性判断 如果是第1次最后1位开始 那么不应该判断障碍物连续性 因为这次位图都是无效的
			if loopTimes != 0 || fatherY%8 != 7 {
				NLastIsObstacle = 1&nowBitN != 0
			}
			//----------------------------------------更新位图loopTimes++ bitStartIndex += delta 注意位图越界
			loopTimes++
			bitStartIndex += delta
			if bitStartIndex >= bitMapLen { //本次位图已经遍历完没找到跳点
				return nil, false
			}
			nowBitM = bitMap[fatherY][bitStartIndex]
			nowBitN = bitMap[fatherY-1][bitStartIndex]
		}
	}
	//默认未找到跳点
	return nil, false
}

// 1次方法调用即可获得跳点 从垂直方向获取当前8位可能的跳点 index是预存的位图索引 delta是当前方向正负 以下x+或右y+为正
func (m *Map) get_Up_VerticalRealJumpPoint(availableP, availableN, fatherIsDiagonal bool, fatherNode *Node) (*Node, bool) {
	//定义位图 记录位图遍历的次数 记录当前bit --
	delta := -1
	fatherX, fatherY := fatherNode.X, fatherNode.Y
	var bitMap [][]uint8
	var loopTimes, PJumpTimes, NJumpTimes = 0, 0, 0
	var nowBitP, nowBitM, nowBitN uint8 = 0, 0, 0
	var bitStartIndex = fatherX / 8
	var MZeroTimes = -1
	var PLastIsObstacle, NLastIsObstacle = false, false
	var mayEndPointLoopTimes = m.calVEndPointLoopTimes(fatherNode, false)
	var temJumpNode *Node = nil
	var temJumpNewDir uint8 = 0
	var mayFindJumpNode = false
	//===============================Loop
	bitMap = m.UpToDownColsBitMap
	//1.循环获取可能的跳点
	if availableP && availableN {
		//2.第1次先清除需要的当前8位部分左边所有位 置为0
		nowBitP = clearRightBits(bitMap[fatherY+1][bitStartIndex], 8-(fatherX%8))
		nowBitM = clearRightBits(bitMap[fatherY][bitStartIndex], 8-(fatherX%8))
		nowBitN = clearRightBits(bitMap[fatherY-1][bitStartIndex], 8-(fatherX%8))
		//--------------------------------------------------------PMN主循环
		for {
			//有障碍物连续性的上1位 位图底必然是障碍 当前位不是障碍物时,即视为找到跳点的下一个 -- 就是找后导1的数量 特殊：全部都是障碍 后导1数量为8
			//无障碍物连续性的上1位 位图必然全是非障碍 正常位运算自己
			//P
			if PLastIsObstacle {
				PJumpTimes = calTrailingOne_8bit(nowBitP)
			} else {
				PJumpTimes = calTrailingZero_8bit((nowBitP << 1) & ^nowBitP)
			}
			//N
			if NLastIsObstacle {
				NJumpTimes = calTrailingOne_8bit(nowBitN)
			} else {
				NJumpTimes = calTrailingZero_8bit((nowBitN << 1) & ^nowBitN)
			}
			//M 只需要看障碍物位置
			MZeroTimes = calTrailingZero_8bit(nowBitM)
			//---------------------------------------跳点分情况判断
			if mayEndPointLoopTimes == loopTimes { //0.(单独加1个)如果当前处于终点位图 那么回退原始方法 不管结果如何 都要结束 因为终点位图永远都是最后一次遍历
				if mayEndPointLoopTimes == 0 { //起始位图就是终点位图 起始点是fatherNode的下1个点 当前方向 Left
					return m.doVMayEndPointBitJumpNode(fatherX-1, fatherY, fatherNode, -1, availableP, availableN, fatherIsDiagonal)
				} else { //终点位图不是起始位图 起始点为当前位图最后1个再下1个 也就是上1个位图的第1个 当前方向Right
					return m.doVMayEndPointBitJumpNode((((fatherX/8)-loopTimes)*8)+7+1, fatherY, fatherNode, -1, availableP, availableN, fatherIsDiagonal)
				}
			}
			if PJumpTimes < NJumpTimes { //P小
				//-------------------------------------1.P跳点有效
				if MZeroTimes >= PJumpTimes+1 {
					temJumpNode = m.Nodes[(((fatherX/8)+1-loopTimes)*8)-PJumpTimes][fatherY]
					temJumpNewDir = Bit0 | Bit4
					mayFindJumpNode = true
				} else {
					return nil, false //本次逻辑没有跳点 退出逻辑
				}
			} else if PJumpTimes > NJumpTimes { //N小
				//-------------------------------------2.N跳点有效
				if MZeroTimes >= NJumpTimes+1 {
					temJumpNode = m.Nodes[(((fatherX/8)+1-loopTimes)*8)-NJumpTimes][fatherY]
					temJumpNewDir = Bit0 | Bit7
					mayFindJumpNode = true
				} else {
					return nil, false //本次逻辑没有跳点 退出逻辑
				}
			} else { //PN相等
				//-------------------------------------3.PN次数相同
				if PJumpTimes == 8 { //PN都无跳点
					if MZeroTimes != 8 {
						return nil, false //本次逻辑没有跳点 退出逻辑
					}
				} else {
					if MZeroTimes >= PJumpTimes+1 { //PN跳点都有效
						temJumpNode = m.Nodes[(((fatherX/8)+1-loopTimes)*8)-PJumpTimes][fatherY]
						temJumpNewDir = Bit0 | Bit4 | Bit7
						mayFindJumpNode = true
					} else {
						return nil, false //本次逻辑没有跳点 退出逻辑
					}
				}
			}
			//如果之前的操作找到了可能性跳点或强制邻居
			if mayFindJumpNode {
				if fatherIsDiagonal { //父节点是斜向点拓展探索 父节点必定不在open或close中
					if temJumpNode.isInOpenList1() || temJumpNode.isInCloseList1() { //当前强制邻居在open中 父节点不在open或close中 不需要回溯,当前强制在close中也跳过 并且跳过 --位运算要以当前强制邻居为虚拟父节点继续遍历
						m.reset_Up_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else { //否则强制邻居生效 但是跳点是斜向点本身 不是强制邻居 返回nil,找到有效强制邻居true
						return nil, true
					}
				} else { //父节点是第一级源点 那么父节点必定处于close中
					if temJumpNode.isInOpenList1() { //跳点处于open 回溯当前跳点 跳过
						m.fixRayCastNode_InOpen_New(temJumpNode, fatherNode)                                                                                                                                                                       //回溯
						m.reset_Up_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else if temJumpNode.isInCloseList1() { //跳点处于close中 不回溯 跳过
						m.reset_Up_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else { //跳点不处于open和close 跳点有效 加入open
						return m.addJumpNode_ToOpen_New(fatherNode, temJumpNode, temJumpNewDir) //添加open 新方向
					}
				}
			}
			//---------------------------------------PN障碍连续性判断 如果父节点是第1次最后1位开始(反向除8余0是最后1位) 那么不应该判断障碍物连续性 因为这次位图都是无效的
			if loopTimes != 0 || fatherY%8 != 0 {
				PLastIsObstacle = 128&nowBitP != 0
				NLastIsObstacle = 128&nowBitN != 0
			}
			//----------------------------------------更新位图loopTimes++ bitStartIndex += delta 注意位图越界
			loopTimes++
			bitStartIndex += delta
			if bitStartIndex < 0 { //本次位图已经遍历完没找到跳点
				return nil, false
			}
			nowBitP = bitMap[fatherY+1][bitStartIndex]
			nowBitM = bitMap[fatherY][bitStartIndex]
			nowBitN = bitMap[fatherY-1][bitStartIndex]
		}
	} else if availableP {
		//2.第1次先清除需要的当前8位部分左边所有位 置为0
		nowBitP = clearRightBits(bitMap[fatherY+1][bitStartIndex], 8-(fatherX%8))
		nowBitM = clearRightBits(bitMap[fatherY][bitStartIndex], 8-(fatherX%8))
		//--------------------------------------------------------PM主循环
		for {
			//有障碍物连续性的上1位 位图底必然是障碍 当前位不是障碍物时,即视为找到跳点的下一个 -- 就是找后导1的数量 特殊：全部都是障碍 后导1数量为8
			//无障碍物连续性的上1位 位图必然全是非障碍 正常位运算自己
			//P
			if PLastIsObstacle {
				PJumpTimes = calTrailingOne_8bit(nowBitP)
			} else {
				PJumpTimes = calTrailingZero_8bit((nowBitP << 1) & ^nowBitP)
			}
			//M 只需要看障碍物位置
			MZeroTimes = calTrailingZero_8bit(nowBitM)
			//---------------------------------------跳点分情况判断
			if mayEndPointLoopTimes == loopTimes { //0.(单独加1个)如果当前处于终点位图 那么回退原始方法 不管结果如何 都要结束 因为终点位图永远都是最后一次遍历
				if mayEndPointLoopTimes == 0 { //起始位图就是终点位图 起始点是fatherNode的下1个点 当前方向 Left
					return m.doVMayEndPointBitJumpNode(fatherX-1, fatherY, fatherNode, -1, availableP, availableN, fatherIsDiagonal)
				} else { //终点位图不是起始位图 起始点为当前位图最后1个再下1个 也就是上1个位图的第1个 当前方向Right
					return m.doVMayEndPointBitJumpNode((((fatherX/8)-loopTimes)*8)+7+1, fatherY, fatherNode, -1, availableP, availableN, fatherIsDiagonal)
				}
			}
			if PJumpTimes == 8 { //P无跳点
				//---------------------------------------1.M已有障碍
				if MZeroTimes != 8 {
					return nil, false //本次逻辑没有跳点 退出逻辑
				}
			} else {
				//---------------------------------------2.P跳点有效
				if MZeroTimes >= PJumpTimes+1 { //P跳点都有效
					temJumpNode = m.Nodes[(((fatherX/8)+1-loopTimes)*8)-PJumpTimes][fatherY]
					temJumpNewDir = Bit0 | Bit4
					mayFindJumpNode = true
				} else {
					return nil, false //本次逻辑没有跳点 退出逻辑
				}
			}
			//如果之前的操作找到了可能性跳点或强制邻居
			if mayFindJumpNode {
				if fatherIsDiagonal { //父节点是斜向点拓展探索 父节点必定不在open或close中
					if temJumpNode.isInOpenList1() || temJumpNode.isInCloseList1() { //当前强制邻居在open中 父节点不在open或close中 不需要回溯,当前强制在close中也跳过 并且跳过 --位运算要以当前强制邻居为虚拟父节点继续遍历
						m.reset_Up_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else { //否则强制邻居生效 但是跳点是斜向点本身 不是强制邻居 返回nil,找到有效强制邻居true
						return nil, true
					}
				} else { //父节点是第一级源点 那么父节点必定处于close中
					if temJumpNode.isInOpenList1() { //跳点处于open 回溯当前跳点 跳过
						m.fixRayCastNode_InOpen_New(temJumpNode, fatherNode)                                                                                                                                                                       //回溯
						m.reset_Up_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else if temJumpNode.isInCloseList1() { //跳点处于close中 不回溯 跳过
						m.reset_Up_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else { //跳点不处于open和close 跳点有效 加入open
						return m.addJumpNode_ToOpen_New(fatherNode, temJumpNode, temJumpNewDir) //添加open 新方向
					}
				}
			}
			//---------------------------------------PN障碍连续性判断 如果父节点是第1次最后1位开始(反向除8余0是最后1位) 那么不应该判断障碍物连续性 因为这次位图都是无效的
			if loopTimes != 0 || fatherY%8 != 0 {
				PLastIsObstacle = 128&nowBitP != 0
			}
			//----------------------------------------更新位图loopTimes++ bitStartIndex += delta 注意位图越界
			loopTimes++
			bitStartIndex += delta
			if bitStartIndex < 0 { //本次位图已经遍历完没找到跳点
				return nil, false
			}
			nowBitP = bitMap[fatherY+1][bitStartIndex]
			nowBitM = bitMap[fatherY][bitStartIndex]
		}
	} else if availableN {
		//2.第1次先清除需要的当前8位部分左边所有位 置为0
		nowBitM = clearRightBits(bitMap[fatherY][bitStartIndex], 8-(fatherX%8))
		nowBitN = clearRightBits(bitMap[fatherY-1][bitStartIndex], 8-(fatherX%8))
		//--------------------------------------------------------PN主循环
		for {
			//有障碍物连续性的上1位 位图底必然是障碍 当前位不是障碍物时,即视为找到跳点的下一个 -- 就是找后导1的数量 特殊：全部都是障碍 后导1数量为8
			//无障碍物连续性的上1位 位图必然全是非障碍 正常位运算自己
			//N
			if NLastIsObstacle {
				NJumpTimes = calTrailingOne_8bit(nowBitN)
			} else {
				NJumpTimes = calTrailingZero_8bit((nowBitN << 1) & ^nowBitN)
			}
			//M 只需要看障碍物位置
			MZeroTimes = calTrailingZero_8bit(nowBitM)
			//---------------------------------------跳点分情况判断
			if mayEndPointLoopTimes == loopTimes { //0.(单独加1个)如果当前处于终点位图 那么回退原始方法 不管结果如何 都要结束 因为终点位图永远都是最后一次遍历
				if mayEndPointLoopTimes == 0 { //起始位图就是终点位图 起始点是fatherNode的下1个点 当前方向 Left
					return m.doVMayEndPointBitJumpNode(fatherX-1, fatherY, fatherNode, -1, availableP, availableN, fatherIsDiagonal)
				} else { //终点位图不是起始位图 起始点为当前位图最后1个再下1个 也就是上1个位图的第1个 当前方向Right
					return m.doVMayEndPointBitJumpNode((((fatherX/8)-loopTimes)*8)+7+1, fatherY, fatherNode, -1, availableP, availableN, fatherIsDiagonal)
				}
			}
			if NJumpTimes == 8 { //N无跳点
				//---------------------------------------1.M已有障碍
				if MZeroTimes != 8 {
					return nil, false //本次逻辑没有跳点 退出逻辑
				}
			} else {
				//---------------------------------------2.N跳点有效
				if MZeroTimes >= NJumpTimes+1 { //N跳点都有效
					temJumpNode = m.Nodes[(((fatherX/8)+1-loopTimes)*8)-NJumpTimes][fatherY]
					temJumpNewDir = Bit0 | Bit7
					mayFindJumpNode = true
				} else {
					return nil, false //本次逻辑没有跳点 退出逻辑
				}
			}
			//如果之前的操作找到了可能性跳点或强制邻居
			if mayFindJumpNode {
				if fatherIsDiagonal { //父节点是斜向点拓展探索 父节点必定不在open或close中
					if temJumpNode.isInOpenList1() || temJumpNode.isInCloseList1() { //当前强制邻居在open中 父节点不在open或close中 不需要回溯,当前强制在close中也跳过 并且跳过 --位运算要以当前强制邻居为虚拟父节点继续遍历
						m.reset_Up_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else { //否则强制邻居生效 但是跳点是斜向点本身 不是强制邻居 返回nil,找到有效强制邻居true
						return nil, true
					}
				} else { //父节点是第一级源点 那么父节点必定处于close中
					if temJumpNode.isInOpenList1() { //跳点处于open 回溯当前跳点 跳过
						m.fixRayCastNode_InOpen_New(temJumpNode, fatherNode)                                                                                                                                                                       //回溯
						m.reset_Up_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else if temJumpNode.isInCloseList1() { //跳点处于close中 不回溯 跳过
						m.reset_Up_TemData(temJumpNode, &fatherX, &fatherY, &bitStartIndex, &mayEndPointLoopTimes, &loopTimes, &nowBitN, &nowBitM, &nowBitP, &PLastIsObstacle, &NLastIsObstacle, &mayFindJumpNode, availableN, availableP, bitMap) //重置
						continue
					} else { //跳点不处于open和close 跳点有效 加入open
						return m.addJumpNode_ToOpen_New(fatherNode, temJumpNode, temJumpNewDir) //添加open 新方向
					}
				}
			}
			//---------------------------------------PN障碍连续性判断 如果父节点是第1次最后1位开始(反向除8余0是最后1位) 那么不应该判断障碍物连续性 因为这次位图都是无效的
			if loopTimes != 0 || fatherY%8 != 0 {
				NLastIsObstacle = 128&nowBitN != 0
			}
			//----------------------------------------更新位图loopTimes++ bitStartIndex += delta 注意位图越界
			loopTimes++
			bitStartIndex += delta
			if bitStartIndex < 0 { //本次位图已经遍历完没找到跳点
				return nil, false
			}
			nowBitM = bitMap[fatherY][bitStartIndex]
			nowBitN = bitMap[fatherY-1][bitStartIndex]
		}
	}
	//默认未找到跳点
	return nil, false
}

// 暂时没想到斜边的位运算 暂时使用老方法遍历节点逻辑判断 ---斜边只有在碰到斜边本身的跳点才会结束跳跃
// 可以先获取当前斜边本身的跳点 有几种情况--1.没有斜跳点(遍历到底了 或 某1次跳跃被双边障碍物挡住) 2.有斜跳点(只选择最近的单边斜跳点)
func (m *Map) get_RightUp_DiagonalRealJumpPoint(originalNode *Node) {
	//右上 遍历原地图 DynamicFather Neighbor Obstacle
	var dynamicMidNode = originalNode //dynamicMidNode = m.Nodes[dynamicFatherX][dynamicFatherY]
	var neighborNode *Node = nil      //neighborNode = m.Nodes[dynamicFatherX][dynamicFatherY]
	var ObstacleNode1, ObstacleNode2 *Node = nil, nil
	var neighborIsMapNodeAndNotWall = false
	var dynamicMidNodeHFind, dynamicMidNodeVFind = false, false
	rows := m.Rows
	cols := m.Cols
	dynamicFatherX, dynamicFatherY := originalNode.X, originalNode.Y
	deltaX, deltaY := -1, 1
	//===LOOP 有斜向点 先检查斜向点,让新父类链接根父。再逻辑水平垂直点,就算有水平垂直点，链接新父类也是正确的(水平垂直点检查新父类只会调用1级父类方法检查)
	for {
		//0  0 0
		//O1 N 0
		//D O2 0
		//1.判断当前是不是终点(当前不是下一个斜点) 是的话提前结束 也不用水平垂直了 终点一定不是rootNode点本身 因为终点被open弹出的时候不会进入这里
		if dynamicMidNode == m.endNode {
			m.checkJumpNode_New(originalNode, dynamicMidNode, 0) //方向默认没有就行 不需要继续跳跃
			return
		}
		//2.判断dynamicMidNode在不在open中 不在才水平垂直拓展 在则跳过 进行下一个斜向点的检索
		if !dynamicMidNode.isInOpenList1() && !dynamicMidNode.isInCloseList1() && originalNode != dynamicMidNode { //不在open和close中 且 不是斜向点本身 才进行水平垂直的拓展
			//3.先水平垂直 水平垂直的没写好 第一次跳点还不是水平垂直点！！！！
			_, dynamicMidNodeHFind = m.get_Right_HorizontalRealJumpPoint(dynamicFatherX+1 < rows, dynamicFatherX-1 >= 0, true, dynamicMidNode) //右
			_, dynamicMidNodeVFind = m.get_Up_VerticalRealJumpPoint(dynamicFatherY+1 < cols, dynamicFatherY-1 >= 0, true, dynamicMidNode)      //上
			if dynamicMidNodeHFind && dynamicMidNodeVFind {                                                                                    //水平垂直都有强制邻居 跳点加入水平垂直方向
				m.addJumpNode_ToOpen_New(originalNode, dynamicMidNode, Bit4|Bit0|Bit1) //右上 右 上
				return
			} else if dynamicMidNodeHFind {
				m.addJumpNode_ToOpen_New(originalNode, dynamicMidNode, Bit4|Bit1) //右上 右
				return
			} else if dynamicMidNodeVFind {
				m.addJumpNode_ToOpen_New(originalNode, dynamicMidNode, Bit4|Bit0) //右上  上
				return
			}
		} else { //斜向点在open中 在open中才进行当前父类的Fix修复
			if dynamicMidNode.isInOpenList1() {
				m.fixRayCastNode_InOpen_New(dynamicMidNode, originalNode)
			}
		}
		//4.再判断斜向 邻居 那么可能性墙也在界内
		if neighborNode, neighborIsMapNodeAndNotWall = m.isMapNodeAndNotWall00(dynamicFatherX+deltaX, dynamicFatherY+deltaY); neighborIsMapNodeAndNotWall {
			ObstacleNode1 = m.Nodes[neighborNode.X][neighborNode.Y-1]
			ObstacleNode2 = m.Nodes[neighborNode.X+1][neighborNode.Y]
			//5.判断可能性墙 2边同时是墙则停止 最多只有1边是墙 也就是最多只有1边有新的斜向
			if ObstacleNode1.IsWall() && ObstacleNode2.IsWall() { //都是墙 无跳点
				return
			} else if ObstacleNode1.IsWall() && m.isMapNodeAndNotWall01(neighborNode.X-1, neighborNode.Y-1) && m.isMapNodeAndNotWall01(neighborNode.X-1, neighborNode.Y) { //父上是墙 找到跳点
				//看邻居跳点是否在open 或 close 或 都不在 作不同处理
				if neighborNode.isInOpenList1() { //在open中 fix 然后继续下一个
					m.fixRayCastNode_InOpen_New(neighborNode, originalNode)
				} else if neighborNode.isInCloseList1() { //在close中 继续下一个

				} else { //不在open 和 close 添加进open 停止本次斜向逻辑
					m.addJumpNode_ToOpen_New(originalNode, neighborNode, Bit4|Bit7|Bit0|Bit1) //原方向右上 新方向左上 原方向拆解水平垂直 右 上(因为斜向点本身在斜向的时候不会水平垂直拓展自己)
					return
				}
			} else if ObstacleNode2.IsWall() && m.isMapNodeAndNotWall01(neighborNode.X+1, neighborNode.Y+1) && m.isMapNodeAndNotWall01(neighborNode.X, neighborNode.Y+1) { //父右是墙 找到跳点
				//看邻居跳点是否在open 或 close 或 都不在 作不同处理
				if neighborNode.isInOpenList1() { //在open中 fix 然后继续下一个
					m.fixRayCastNode_InOpen_New(neighborNode, originalNode)
				} else if neighborNode.isInCloseList1() { //在close中 继续下一个

				} else { //不在open 和 close 添加进open 停止本次斜向逻辑
					m.addJumpNode_ToOpen_New(originalNode, neighborNode, Bit4|Bit5|Bit0|Bit1) //原方向右上 新方向右下 原方向拆解水平垂直 右 上(因为斜向点本身在斜向的时候不会水平垂直拓展自己)
					return
				}
			}
		} else {
			return
		}
		//2.变换下一新父节点
		dynamicMidNode = neighborNode
		dynamicFatherX = dynamicMidNode.X
		dynamicFatherY = dynamicMidNode.Y
	}
}

// 可以先获取当前斜边本身的跳点 有几种情况--1.没有斜跳点(遍历到底了 或 某1次跳跃被双边障碍物挡住) 2.有斜跳点(只选择最近的单边斜跳点)
func (m *Map) get_RightDown_DiagonalRealJumpPoint(originalNode *Node) {
	//右下 遍历原地图 DynamicFather Neighbor Obstacle
	var dynamicMidNode = originalNode //dynamicMidNode = m.Nodes[dynamicFatherX][dynamicFatherY]
	var neighborIsMapNodeAndNotWall = false
	var neighborNode *Node = nil //neighborNode = m.Nodes[dynamicFatherX][dynamicFatherY]
	var ObstacleNode1, ObstacleNode2 *Node = nil, nil
	var dynamicMidNodeHFind, dynamicMidNodeVFind = false, false
	rows := m.Rows
	cols := m.Cols
	dynamicFatherX, dynamicFatherY := originalNode.X, originalNode.Y
	deltaX, deltaY := 1, 1
	//===LOOP
	for {
		//D O1  0
		//O2 N	0
		//0  0  0
		//1.判断当前是不是终点(当前不是下一个斜点) 是的话提前结束 也不用水平垂直了 终点一定不是rootNode点本身 因为终点被open弹出的时候不会进入这里
		if dynamicMidNode == m.endNode {
			m.checkJumpNode_New(originalNode, dynamicMidNode, 0) //方向默认没有就行 不需要继续判断了
			return
		}
		//2.判断dynamicMidNode在不在open中 不在才水平垂直拓展 在则跳过 进行下一个斜向点的检索
		if !dynamicMidNode.isInOpenList1() && !dynamicMidNode.isInCloseList1() && originalNode != dynamicMidNode { //不在open和close中 且 不是斜向点本身 才进行水平垂直的拓展
			//3.先水平垂直 水平垂直的没写好 第一次跳点还不是水平垂直点！！！！
			_, dynamicMidNodeHFind = m.get_Right_HorizontalRealJumpPoint(dynamicFatherX+1 < rows, dynamicFatherX-1 >= 0, true, dynamicMidNode) //右
			_, dynamicMidNodeVFind = m.get_Down_VerticalRealJumpPoint(dynamicFatherY+1 < cols, dynamicFatherY-1 >= 0, true, dynamicMidNode)    //下
			if dynamicMidNodeHFind && dynamicMidNodeVFind {                                                                                    //水平垂直都有强制邻居 跳点加入水平垂直方向
				m.addJumpNode_ToOpen_New(originalNode, dynamicMidNode, Bit5|Bit1|Bit2) //右下 右 下
				return
			} else if dynamicMidNodeHFind {
				m.addJumpNode_ToOpen_New(originalNode, dynamicMidNode, Bit5|Bit1) //右下 右
				return
			} else if dynamicMidNodeVFind {
				m.addJumpNode_ToOpen_New(originalNode, dynamicMidNode, Bit5|Bit2) //右下 下
				return
			}
		} else { //斜向点在open中 在open中才进行当前父类的Fix修复
			if dynamicMidNode.isInOpenList1() {
				m.fixRayCastNode_InOpen_New(dynamicMidNode, originalNode)
			}
		}
		//2.再斜向
		if neighborNode, neighborIsMapNodeAndNotWall = m.isMapNodeAndNotWall00(dynamicFatherX+deltaX, dynamicFatherY+deltaY); neighborIsMapNodeAndNotWall {
			ObstacleNode1 = m.Nodes[neighborNode.X-1][neighborNode.Y]
			ObstacleNode2 = m.Nodes[neighborNode.X][neighborNode.Y-1]
			//3.判断可能性墙
			if ObstacleNode1.IsWall() && ObstacleNode2.IsWall() { //都是墙 无跳点
				return
			} else if ObstacleNode1.IsWall() && m.isMapNodeAndNotWall01(neighborNode.X-1, neighborNode.Y+1) && m.isMapNodeAndNotWall01(neighborNode.X, neighborNode.Y+1) { //只有右是墙 找到跳点
				//看邻居跳点是否在open 或 close 或 都不在 作不同处理
				if neighborNode.isInOpenList1() { //在open中 fix 然后继续下一个
					m.fixRayCastNode_InOpen_New(neighborNode, originalNode)
				} else if neighborNode.isInCloseList1() { //在close中 继续下一个

				} else { //不在open 和 close 添加进open 停止本次斜向逻辑
					m.addJumpNode_ToOpen_New(originalNode, neighborNode, Bit5|Bit4|Bit1|Bit2) //原方向右下 新方向右上 原方向拆解水平垂直 右 下(因为斜向点本身在斜向的时候不会水平垂直拓展自己)
					return
				}
			} else if ObstacleNode2.IsWall() && m.isMapNodeAndNotWall01(neighborNode.X+1, neighborNode.Y-1) && m.isMapNodeAndNotWall01(neighborNode.X+1, neighborNode.Y) { //只有下是墙 找到跳点
				//看邻居跳点是否在open 或 close 或 都不在 作不同处理
				if neighborNode.isInOpenList1() { //在open中 fix 然后继续下一个
					m.fixRayCastNode_InOpen_New(neighborNode, originalNode)
				} else if neighborNode.isInCloseList1() { //在close中 继续下一个

				} else { //不在open 和 close 添加进open 停止本次斜向逻辑
					m.addJumpNode_ToOpen_New(originalNode, neighborNode, Bit5|Bit6|Bit1|Bit2) //原方向右下 新方向左下 原方向拆解水平垂直 右 下(因为斜向点本身在斜向的时候不会水平垂直拓展自己)
					return
				}
			}
		} else {
			return
		}
		//2.变换下一新父节点
		dynamicMidNode = neighborNode
		dynamicFatherX = dynamicMidNode.X
		dynamicFatherY = dynamicMidNode.Y
	}
}

// 可以先获取当前斜边本身的跳点 有几种情况--1.没有斜跳点(遍历到底了 或 某1次跳跃被双边障碍物挡住) 2.有斜跳点(只选择最近的单边斜跳点)
func (m *Map) get_LeftDown_DiagonalRealJumpPoint(originalNode *Node) {
	//左下 遍历原地图 DynamicFather Neighbor Obstacle
	var dynamicMidNode = originalNode //dynamicMidNode = m.Nodes[dynamicFatherX][dynamicFatherY]
	var neighborIsMapNodeAndNotWall = false
	var neighborNode *Node = nil //neighborNode = m.Nodes[dynamicFatherX][dynamicFatherY]
	var ObstacleNode1, ObstacleNode2 *Node = nil, nil
	var dynamicMidNodeHFind, dynamicMidNodeVFind = false, false
	rows := m.Rows
	cols := m.Cols
	dynamicFatherX, dynamicFatherY := originalNode.X, originalNode.Y
	deltaX, deltaY := 1, -1
	//===LOOP
	for {
		//0 O1 D
		//0 N O2
		//0 0 0
		//1.判断当前是不是终点(当前不是下一个斜点) 是的话提前结束 也不用水平垂直了 终点一定不是rootNode点本身 因为终点被open弹出的时候不会进入这里
		if dynamicMidNode == m.endNode {
			m.checkJumpNode_New(originalNode, dynamicMidNode, 0) //方向默认没有就行 不需要继续判断了
			return
		}
		//2.判断dynamicMidNode在不在open中 不在才水平垂直拓展 在则跳过 进行下一个斜向点的检索
		if !dynamicMidNode.isInOpenList1() && !dynamicMidNode.isInCloseList1() && originalNode != dynamicMidNode { //不在open和close中 且 不是斜向点本身 才进行水平垂直的拓展
			//3.先水平垂直 水平垂直的没写好 第一次跳点还不是水平垂直点！！！！
			_, dynamicMidNodeHFind = m.get_Left_HorizontalRealJumpPoint(dynamicFatherX+1 < rows, dynamicFatherX-1 >= 0, true, dynamicMidNode) //左
			_, dynamicMidNodeVFind = m.get_Down_VerticalRealJumpPoint(dynamicFatherY+1 < cols, dynamicFatherY-1 >= 0, true, dynamicMidNode)   //下
			if dynamicMidNodeHFind && dynamicMidNodeVFind {                                                                                   //水平垂直都有强制邻居 跳点加入水平垂直方向
				m.addJumpNode_ToOpen_New(originalNode, dynamicMidNode, Bit6|Bit3|Bit2) //左下 左 下
				return
			} else if dynamicMidNodeHFind {
				m.addJumpNode_ToOpen_New(originalNode, dynamicMidNode, Bit6|Bit3) //左下  左
				return
			} else if dynamicMidNodeVFind {
				m.addJumpNode_ToOpen_New(originalNode, dynamicMidNode, Bit6|Bit2) //左下  下
				return
			}
		} else { //斜向点在open中 在open中才进行当前父类的Fix修复
			if dynamicMidNode.isInOpenList1() {
				m.fixRayCastNode_InOpen_New(dynamicMidNode, originalNode)
			}
		}
		//3.再斜向
		if neighborNode, neighborIsMapNodeAndNotWall = m.isMapNodeAndNotWall00(dynamicFatherX+deltaX, dynamicFatherY+deltaY); neighborIsMapNodeAndNotWall {
			ObstacleNode1 = m.Nodes[neighborNode.X-1][neighborNode.Y]
			ObstacleNode2 = m.Nodes[neighborNode.X][neighborNode.Y+1]
			//3.判断可能性墙
			if ObstacleNode1.IsWall() && ObstacleNode2.IsWall() { //都是墙 无跳点
				return
			} else if ObstacleNode1.IsWall() && m.isMapNodeAndNotWall01(neighborNode.X-1, neighborNode.Y-1) && m.isMapNodeAndNotWall01(neighborNode.X, neighborNode.Y-1) { //父左是墙 找到跳点
				//看邻居跳点是否在open 或 close 或 都不在 作不同处理
				if neighborNode.isInOpenList1() { //在open中 fix 然后继续下一个
					m.fixRayCastNode_InOpen_New(neighborNode, originalNode)
				} else if neighborNode.isInCloseList1() { //在close中 继续下一个

				} else { //不在open 和 close 添加进open 停止本次斜向逻辑
					m.addJumpNode_ToOpen_New(originalNode, neighborNode, Bit6|Bit7|Bit3|Bit2) //原方向左下 新方向左上 原方向拆解水平垂直 左 下(因为斜向点本身在斜向的时候不会水平垂直拓展自己)
					return
				}
			} else if ObstacleNode2.IsWall() && m.isMapNodeAndNotWall01(neighborNode.X+1, neighborNode.Y+1) && m.isMapNodeAndNotWall01(neighborNode.X+1, neighborNode.Y) { //父下是墙 找到跳点
				//看邻居跳点是否在open 或 close 或 都不在 作不同处理
				if neighborNode.isInOpenList1() { //在open中 fix 然后继续下一个
					m.fixRayCastNode_InOpen_New(neighborNode, originalNode)
				} else if neighborNode.isInCloseList1() { //在close中 继续下一个

				} else { //不在open 和 close 添加进open 停止本次斜向逻辑
					m.addJumpNode_ToOpen_New(originalNode, neighborNode, Bit6|Bit5|Bit3|Bit2) //原方向左下 新方向右下 原方向拆解水平垂直 左 下(因为斜向点本身在斜向的时候不会水平垂直拓展自己)
					return
				}
			}
		} else {
			return
		}
		//2.变换下一新父节点
		dynamicMidNode = neighborNode
		dynamicFatherX = dynamicMidNode.X
		dynamicFatherY = dynamicMidNode.Y
	}
}

// 可以先获取当前斜边本身的跳点 有几种情况--1.没有斜跳点(遍历到底了 或 某1次跳跃被双边障碍物挡住) 2.有斜跳点(只选择最近的单边斜跳点)
func (m *Map) get_LeftUp_DiagonalRealJumpPoint(originalNode *Node) {
	//左上 遍历原地图 DynamicFather Neighbor Obstacle
	var dynamicMidNode = originalNode //dynamicMidNode = m.Nodes[dynamicFatherX][dynamicFatherY]
	var neighborIsMapNodeAndNotWall = false
	var neighborNode *Node = nil //neighborNode = m.Nodes[dynamicFatherX][dynamicFatherY]
	var ObstacleNode1, ObstacleNode2 *Node = nil, nil
	var dynamicMidNodeHfind, dynamicMidNodeVfind = false, false
	rows := m.Rows
	cols := m.Cols
	dynamicFatherX, dynamicFatherY := originalNode.X, originalNode.Y
	deltaX, deltaY := -1, -1
	//===LOOP
	for {
		//0 0 0
		//0 N O1
		//0 O2 D
		//1.判断当前是不是终点(当前不是下一个斜点) 是的话提前结束 也不用水平垂直了 终点一定不是rootNode点本身 因为终点被open弹出的时候不会进入这里
		if dynamicMidNode == m.endNode {
			m.checkJumpNode_New(originalNode, dynamicMidNode, 0) //方向默认没有就行 不需要继续判断了
			return
		}
		//2.判断dynamicMidNode在不在open中 不在才水平垂直拓展 在则跳过 进行下一个斜向点的检索
		if !dynamicMidNode.isInOpenList1() && !dynamicMidNode.isInCloseList1() && originalNode != dynamicMidNode { //不在open和close中 且 不是斜向点本身 才进行水平垂直的拓展
			//3.先水平垂直 找到强制邻居马上停止斜向点跳跃
			_, dynamicMidNodeHfind = m.get_Left_HorizontalRealJumpPoint(dynamicFatherX+1 < rows, dynamicFatherX-1 >= 0, true, dynamicMidNode) //左
			_, dynamicMidNodeVfind = m.get_Up_VerticalRealJumpPoint(dynamicFatherY+1 < cols, dynamicFatherY-1 >= 0, true, dynamicMidNode)     //上
			if dynamicMidNodeHfind && dynamicMidNodeVfind {                                                                                   //水平垂直都有强制邻居 跳点加入水平垂直方向
				m.addJumpNode_ToOpen_New(originalNode, dynamicMidNode, Bit7|Bit3|Bit0) //左上 左 上
				return
			} else if dynamicMidNodeHfind {
				m.addJumpNode_ToOpen_New(originalNode, dynamicMidNode, Bit7|Bit3) //左上 左
				return
			} else if dynamicMidNodeVfind {
				m.addJumpNode_ToOpen_New(originalNode, dynamicMidNode, Bit7|Bit0) //左上  上
				return
			}
		} else { //斜向点在open中 在open中才进行当前父类的Fix修复
			if dynamicMidNode.isInOpenList1() {
				m.fixRayCastNode_InOpen_New(dynamicMidNode, originalNode)
			}
		}
		//3.再斜向 新父界内 那么可能性墙也在界内
		if neighborNode, neighborIsMapNodeAndNotWall = m.isMapNodeAndNotWall00(dynamicFatherX+deltaX, dynamicFatherY+deltaY); neighborIsMapNodeAndNotWall {
			ObstacleNode1 = m.Nodes[neighborNode.X][neighborNode.Y+1]
			ObstacleNode2 = m.Nodes[neighborNode.X+1][neighborNode.Y]
			//3.判断可能性墙
			if ObstacleNode1.IsWall() && ObstacleNode2.IsWall() { //都是墙 无跳点
				return
			} else if ObstacleNode1.IsWall() && m.isMapNodeAndNotWall01(neighborNode.X-1, neighborNode.Y+1) && m.isMapNodeAndNotWall01(neighborNode.X-1, neighborNode.Y) { //只有上是墙 找到跳点
				//看邻居跳点是否在open 或 close 或 都不在 作不同处理
				if neighborNode.isInOpenList1() { //在open中 fix 然后继续下一个
					m.fixRayCastNode_InOpen_New(neighborNode, originalNode)
				} else if neighborNode.isInCloseList1() { //在close中 继续下一个

				} else { //不在open 和 close 添加进open 停止本次斜向逻辑
					m.addJumpNode_ToOpen_New(originalNode, neighborNode, Bit7|Bit4|Bit3|Bit0) //原方向左上 新方向右上 原方向拆解水平垂直 左 上(因为斜向点本身在斜向的时候不会水平垂直拓展自己)
					return
				}
			} else if ObstacleNode2.IsWall() && m.isMapNodeAndNotWall01(neighborNode.X+1, neighborNode.Y-1) && m.isMapNodeAndNotWall01(neighborNode.X, neighborNode.Y-1) { //只有左是墙 找到跳点
				//看邻居跳点是否在open 或 close 或 都不在 作不同处理
				if neighborNode.isInOpenList1() { //在open中 fix 然后继续下一个
					m.fixRayCastNode_InOpen_New(neighborNode, originalNode)
				} else if neighborNode.isInCloseList1() { //在close中 继续下一个

				} else { //不在open 和 close 添加进open 停止本次斜向逻辑
					m.addJumpNode_ToOpen_New(originalNode, neighborNode, Bit7|Bit6|Bit3|Bit0) //原方向左上 新方向左下 原方向拆解水平垂直 左 上(因为斜向点本身在斜向的时候不会水平垂直拓展自己)
					return
				}
			}
		} else {
			return
		}
		//2.变换下一新父节点
		dynamicMidNode = neighborNode
		dynamicFatherX = dynamicMidNode.X
		dynamicFatherY = dynamicMidNode.Y
	}
}

// 清除左边连续位数
func clearLeftBits(needClearBit uint8, clearTimes int) uint8 {
	//如 11000010,使得前3位都置为0,后5位不变---则需要掩码为00011111 与 原bit进行与运算
	//而00011111 是 00100000 -1 即为32 - 1
	var mask uint8 = (128 >> (clearTimes - 1)) - 1
	return mask & needClearBit
}

// 清除右边连续位数
func clearRightBits(needClearBit uint8, clearTimes int) uint8 {
	//如 11000010,使得后2位都置为0,前6位不变---则需要掩码为11111100 与 原bit进行与运算
	//而11111100 是00000011所有位取反 而00000011 是00000100 -1 即为4 - 1
	var mask uint8 = (1 << clearTimes) - 1
	return ^mask & needClearBit //注意是取反的mask
}

// 计算8位前导0的数量
func calLeadingZero_8bit(nowBits uint8) int {
	return bits.LeadingZeros8(nowBits)
}

// 计算8位后导0的数量
func calTrailingZero_8bit(nowBits uint8) int {
	return bits.TrailingZeros8(nowBits)
}

// 计算8位前导1的数量
func calLeadingOne_8bit(nowBits uint8) int {
	return bits.LeadingZeros8(^nowBits)
}

// 计算8位后导1的数量
func calTrailingOne_8bit(nowBits uint8) int {
	return bits.TrailingZeros8(^nowBits)
}

// -----------------------------------------------------------------------aPathFind
// aPathFind 实现A星寻路算法，返回是否找到路径
func (m *Map) aPathFind(startX, startY, endX, endY int) bool {
	startNode := m.Nodes[startX][startY]
	endNode := m.Nodes[endX][endY]
	//times := 0
	m.startNode = startNode
	m.endNode = endNode
	startNode.G = 0
	startNode.F = 0
	startNode.signInOpenList1()
	Push(&m.openList1, startNode)
	m.pushTimes++
	//tmp data
	newX, newY := 0, 0
	var neighbor, fatherNode, judgeWallNodeFirst, judgeWallNodeSecond *Node
	i := 0
	var newG, newF float32
	// A LOOP
	for len(m.openList1) > 0 {
		m.popTimes++
		// 从开放列表中获取F值最小的节点
		fatherNode = Pop(&m.openList1)
		// 标记当前父节点处于关闭列表 加入到关闭列表 并且清除开启列表标记 --重要！！因为ResetMap的时候 是从openList和closeList中Reset的
		fatherNode.signInCloseList1_ClearOpenFlag1()
		m.closeList1 = append(m.closeList1, fatherNode)
		// 检查是否到达终点
		if fatherNode == endNode {
			m.oneWayRetracePath(startNode, fatherNode) //当前是反向路径 方法内部需要反转initPath
			return true
		}
		// 检查所有相邻节点
		for i = 0; i < 8; i++ {
			newX = fatherNode.X + eightDirSlice[i][0]
			newY = fatherNode.Y + eightDirSlice[i][1]
			// 检查邻居节点索引是否在地图内
			if !m.isMapNode(newX, newY) {
				//fmt.Println("", "newX:", newX, " newY:", newY)
				continue
			}

			neighbor = m.Nodes[newX][newY]

			// 如果邻居是墙 或者 邻居已经被当做父节点检索之后(已经放入close中了) 那么跳过
			if neighbor.IsWall() || neighbor.isInCloseList1() {
				//fmt.Println("", "neighbor.X:", neighbor.X, " neighbor.Y:", neighbor.Y)
				continue
			}

			//如果邻居已经存在OpenList(节点只会有3种情况 不在open也不在close 只在open 只在close) 如果当前计算出来的新F值小于 自己 当前已存在Open中的F值 那么替换为新的F 并且Fix维护堆性质
			if neighbor.isInOpenList1() {
				//1.当前处于open 计算当前临时的F
				if i >= 4 {
					//	{-1, 1},  //left up 右上邻居 //index 4
					//	{1, 1},   //right up 右下邻居 //index 5
					//	{1, -1},  //right down 左下邻居  //index 6
					//	{-1, -1}, //left down 左上邻居 //index 7
					switch i {
					case 4: //右上邻居 2边是墙则忽略 1边是墙1边是地图外则忽略
						//! N
						//C !
						//2都是墙 忽略
						judgeWallNodeFirst = m.Nodes[fatherNode.X][fatherNode.Y+1]
						judgeWallNodeSecond = m.Nodes[fatherNode.X-1][fatherNode.Y]
					case 5: //右下邻居
						//C !
						//! N
						//2都是墙 忽略
						judgeWallNodeFirst = m.Nodes[fatherNode.X][fatherNode.Y+1]
						judgeWallNodeSecond = m.Nodes[fatherNode.X+1][fatherNode.Y]
					case 6: //左下邻居
						//! C
						//N !
						//2都是墙 忽略
						judgeWallNodeFirst = m.Nodes[fatherNode.X][fatherNode.Y-1]
						judgeWallNodeSecond = m.Nodes[fatherNode.X+1][fatherNode.Y]
					case 7: //左上邻居
						//N !
						//! C
						//2都是墙 忽略
						judgeWallNodeFirst = m.Nodes[fatherNode.X][fatherNode.Y-1]
						judgeWallNodeSecond = m.Nodes[fatherNode.X-1][fatherNode.Y]
					}
					//judgeWall
					if judgeWallNodeFirst.IsWall() && judgeWallNodeSecond.IsWall() {
						continue
					}
					//G:fatherNode.G + math.Sqrt2  H:暂时为空 退化BFS
					//newF = fatherNode.G + math.Sqrt2 + m.diagonalDistance(neighbor.X, neighbor.Y, endNode.X, endNode.Y)
					newG = fatherNode.G + math.Sqrt2
					newF = newG
				} else {
					//G:fatherNode.G + 1  H:暂时为空 退化为迪杰斯特拉
					//newF = fatherNode.G + 1 + m.diagonalDistance(neighbor.X, neighbor.Y, endNode.X, endNode.Y)
					newG = fatherNode.G + 1
					newF = newG
				}
				//2.用来与自己旧的F对比 如果比之前的小就替换 否则什么都不做直接跳过
				if newF < neighbor.F {
					//3.更新为更小的F
					neighbor.G = newG
					neighbor.F = newF
					//4.这么关键的一步怎么能忘了 更新为新的父节点
					neighbor.Parent = fatherNode
					//4.Fix open,索引就是neighbor节点存储的openListIndex
					Fix(&m.openList1, neighbor.IndexInOpenList)
				}
				//End:逻辑结束就跳过
				continue
			}

			//当前邻居不处于open也不处于close 正常逻辑
			if i >= 4 {
				//	{-1, 1},  //left up 右上邻居 //index 4
				//	{1, 1},   //right up 右下邻居 //index 5
				//	{1, -1},  //right down 左下邻居  //index 6
				//	{-1, -1}, //left down 左上邻居 //index 7
				switch i {
				case 4: //右上邻居 2边是墙则忽略 1边是墙1边是地图外则忽略
					//! N
					//C !
					//2都是墙 忽略
					if m.Nodes[fatherNode.X][fatherNode.Y+1].IsWall() && m.Nodes[fatherNode.X-1][fatherNode.Y].IsWall() {
						continue
					}
				case 5: //右下邻居
					//C !
					//! N
					//2都是墙 忽略
					if m.Nodes[fatherNode.X][fatherNode.Y+1].IsWall() && m.Nodes[fatherNode.X+1][fatherNode.Y].IsWall() {
						continue
					}
				case 6: //左下邻居
					//! C
					//N !
					//2都是墙 忽略
					if m.Nodes[fatherNode.X][fatherNode.Y-1].IsWall() && m.Nodes[fatherNode.X+1][fatherNode.Y].IsWall() {
						continue
					}
				case 7: //左上邻居
					//N !
					//! C
					//2都是墙 忽略
					if m.Nodes[fatherNode.X][fatherNode.Y-1].IsWall() && m.Nodes[fatherNode.X-1][fatherNode.Y].IsWall() {
						continue
					}
				}
				neighbor.G = fatherNode.G + math.Sqrt2 // 计算新的G值 斜向
			} else {
				neighbor.G = fatherNode.G + 1 // 计算新的G值 直向
			}
			//fmt.Println("", "neighbor.X:", neighbor.X, " neighbor.Y:", neighbor.Y)
			// 如果新节点不在开放列表中并且也不在关闭列表中 F = G50% + H50%
			neighbor.Parent = fatherNode
			//var nowCurrentToEndDisSqr = float32((neighbor.X-endX)*(neighbor.X-endX) + (neighbor.Y-endY)*(neighbor.Y-endY))
			//neighbor.F = neighbor.G + m.diagonalDistance(neighbor.X, neighbor.Y, endNode.X, endNode.Y) + m.CalculateCross(neighbor)*0.001*(nowCurrentToEndDisSqr/SEdisSqr) //+ m.CalculateCross(neighbor)*0.001
			//neighbor.F = neighbor.G + m.diagonalDistance(neighbor.X, neighbor.Y, endNode.X, endNode.Y) //neighbor已在switch更新
			neighbor.F = neighbor.G
			//加入到开启列表
			neighbor.signInOpenList1()
			Push(&m.openList1, neighbor)
			m.pushTimes++
			//打印openlist
			//for i := 0; i < len(m.openListFirst); i++ {
			//	fmt.Println("openListFirst", "openListFirst.X:", m.openListFirst[i].X, "openListFirst.Y:", m.openListFirst[i].Y)
			//}
			//times++
			neighbor.isOut = true
		}
	}
	fmt.Println("开启列表为空")
	return false
}

// dijieActiveBresenhamPathFind 迪杰邻居与根父节点检查是否可连并更新代价 只要与根父节点直连 直接更新 代价只会小于当前或等于
func (m *Map) dijieActiveBresenhamPathFind(startX, startY, endX, endY int) bool {
	startNode := m.Nodes[startX][startY]
	endNode := m.Nodes[endX][endY]
	//times := 0
	m.startNode = startNode
	m.endNode = endNode
	startNode.G = 0
	startNode.F = 0
	startNode.signInOpenList1()
	Push(&m.openList1, startNode)
	m.pushTimes++
	//tmp data
	i, newX, newY := 0, 0, 0
	var neighbor, fatherNode, tmpRootFatherNode, availableRootFatherNode, judgeWallNodeFirst, judgeWallNodeSecond *Node
	var newG, newF float32
	// dijie LOOP
	for len(m.openList1) > 0 {
		m.popTimes++
		// 从开放列表中获取F值最小的节点
		fatherNode = Pop(&m.openList1)
		// 标记当前父节点处于关闭列表 加入到关闭列表 并且清除开启列表标记 --重要！！因为ResetMap的时候 是从openList和closeList中Reset的
		fatherNode.signInCloseList1_ClearOpenFlag1()
		m.closeList1 = append(m.closeList1, fatherNode)
		// 检查是否到达终点
		if fatherNode == endNode {
			m.oneWayRetracePath(startNode, fatherNode) //当前是反向路径 方法内部需要反转initPath
			return true
		}
		// 检查所有相邻节点
		for i = 0; i < 8; i++ {
			newX = fatherNode.X + eightDirSlice[i][0]
			newY = fatherNode.Y + eightDirSlice[i][1]
			// 检查邻居节点索引是否在地图内
			if !m.isMapNode(newX, newY) {
				//fmt.Println("", "newX:", newX, " newY:", newY)
				continue
			}

			neighbor = m.Nodes[newX][newY]

			// 如果邻居是墙 或者 邻居已经被当做父节点检索之后(已经放入close中了) 那么跳过
			if neighbor.IsWall() || neighbor.isInCloseList1() {
				//fmt.Println("", "neighbor.X:", neighbor.X, " neighbor.Y:", neighbor.Y)
				continue
			}

			//先判断邻居有没有拓展价值 有的话记录临时G
			if i >= 4 {
				//	{-1, 1},  //left up 右上邻居 //index 4
				//	{1, 1},   //right up 右下邻居 //index 5
				//	{1, -1},  //right down 左下邻居  //index 6
				//	{-1, -1}, //left down 左上邻居 //index 7
				switch i {
				case 4: //右上邻居 2边是墙则忽略 1边是墙1边是地图外则忽略
					//! N
					//C !
					//2都是墙 忽略
					judgeWallNodeFirst = m.Nodes[fatherNode.X][fatherNode.Y+1]
					judgeWallNodeSecond = m.Nodes[fatherNode.X-1][fatherNode.Y]
				case 5: //右下邻居
					//C !
					//! N
					//2都是墙 忽略
					judgeWallNodeFirst = m.Nodes[fatherNode.X][fatherNode.Y+1]
					judgeWallNodeSecond = m.Nodes[fatherNode.X+1][fatherNode.Y]
				case 6: //左下邻居
					//! C
					//N !
					//2都是墙 忽略
					judgeWallNodeFirst = m.Nodes[fatherNode.X][fatherNode.Y-1]
					judgeWallNodeSecond = m.Nodes[fatherNode.X+1][fatherNode.Y]
				case 7: //左上邻居
					//N !
					//! C
					//2都是墙 忽略
					judgeWallNodeFirst = m.Nodes[fatherNode.X][fatherNode.Y-1]
					judgeWallNodeSecond = m.Nodes[fatherNode.X-1][fatherNode.Y]
				}
				//judgeWall
				if judgeWallNodeFirst.IsWall() && judgeWallNodeSecond.IsWall() {
					continue
				}
			}
			//================不管在不在open 都要与根父作一次可视化检查 1级是当前父节点(上面判断过了 肯定可以与所有可用邻居直连) 2级是当前父节点的父节点(根父节点)
			//1.注意当前邻居如果不在open中 是没有父节点的 都假设可用父节点就是当前fatherNode
			//2.注意当前邻居如果在open中 必定有自己的父节点 都假设可用父节点就是当前fatherNode
			availableRootFatherNode = fatherNode
			//1.尝试回溯根父节点 当前fatherNode首次进来是起点的情况 那么它的邻居肯定都不在open中
			if fatherNode.Parent != nil { //
				tmpRootFatherNode = fatherNode.Parent
			}
			//2.尝试判断新根节点连通性
			for tmpRootFatherNode != nil {
				if m.JudgeLineObstacleNew(neighbor.X, neighbor.Y, tmpRootFatherNode.X, tmpRootFatherNode.Y) {
					availableRootFatherNode = tmpRootFatherNode  //先更新可用根父节点为第2...n级
					tmpRootFatherNode = tmpRootFatherNode.Parent //尝试下1级根父节点 如果下1级不可以直连 那么availableRootFatherNode记录的就是最后一次可直连的根父
				} else {
					break
				}
			}
			//3.可视化检测结束 当前可用父可能是 fatherNode本身 或者 2级以上根父
			//newG = 当前邻居到可用根父节点的G(直线欧几里得距离)
			newG = m.euclideanDistance(availableRootFatherNode.X, availableRootFatherNode.Y, neighbor.X, neighbor.Y)
			//newF = 可用根父节点本身的G(这里等于fatherNode的F) + newG
			newF = availableRootFatherNode.G + newG
			//-----------------------------------------------------------邻居判断在不在open中
			//如果邻居已经存在OpenList(节点只会有3种情况 不在open也不在close 只在open 只在close) 如果当前计算出来的新F值小于 自己 当前已存在Open中的F值 那么替换为新的F 并且Fix维护堆性质
			if neighbor.isInOpenList1() { //邻居已经有自己的父节点
				//对于当前处于open中的邻居 都是新的父节点 与 自己本身的代价进行比较
				//2.用来与自己旧的F对比 如果比之前的小就替换 否则什么都不做直接跳过
				if newF < neighbor.F {
					//2.注意 g也要更新
					//neighbor.G = availableRootFatherNode.G + newG
					//2.更新新的 上面的迪杰g 等同于 newF
					neighbor.G = newF
					//3.更新为更小的F
					neighbor.F = newF
					//4.更新为新的根父节点
					neighbor.Parent = availableRootFatherNode
					//4.Fix open,索引就是neighbor节点存储的openListIndex
					Fix(&m.openList1, neighbor.IndexInOpenList)
				}
			} else {
				//可视化检测结束 当前可用父可能是 fatherNode本身 或者 2级以上根父
				//1.对于当前不处于open中的邻居 邻居本身没有父节点 不需要比较旧的邻居代价 直接计算当前新的父或者根父节点代价存储 并添加进open
				//2.注意 g也要更新
				//neighbor.G = availableRootFatherNode.G + newG
				//2.更新新的 上面的迪杰g 等同于 newF
				neighbor.G = newF
				//3.更新为更小的F
				neighbor.F = newF
				//4.更新为新的根父节点
				neighbor.Parent = availableRootFatherNode
				//5.Push openList1
				Push(&m.openList1, neighbor)
				//6.记得标记
				neighbor.signInOpenList1()
				m.pushTimes++
			}
			//fmt.Println("", "neighbor.X:", neighbor.X, " neighbor.Y:", neighbor.Y)
			//打印openlist
			//for i := 0; i < len(m.openListFirst); i++ {
			//	fmt.Println("openListFirst", "openListFirst.X:", m.openListFirst[i].X, "openListFirst.Y:", m.openListFirst[i].Y)
			//}
			//times++
			neighbor.isOut = true
		}
	}
	fmt.Println("开启列表为空")
	return false
}

func (m *Map) dijiePassiveBresenhamPathFind(startX, startY, endX, endY int) bool {
	startNode := m.Nodes[startX][startY]
	endNode := m.Nodes[endX][endY]
	//times := 0
	m.startNode = startNode
	m.endNode = endNode
	startNode.G = 0
	startNode.F = 0
	startNode.signInOpenList1()
	Push(&m.openList1, startNode)
	m.pushTimes++
	//tmp data
	i, newX, newY := 0, 0, 0 //tmpRootFatherNode
	var neighbor, fatherNode, tmpRootFatherNode, availableRootFatherNode, judgeWallNodeFirst, judgeWallNodeSecond *Node
	var newG, newF float32
	var tmpG float32
	var needChangeParent = false
	// dijie LOOP
	for len(m.openList1) > 0 {
		m.popTimes++
		// 从开放列表中获取F值最小的节点
		fatherNode = Pop(&m.openList1)
		//----------------------------------------------------只对open弹出来的fatherNode可视化
		//1.如果当前fatherNode不是起点 那么它的父节点一定存在并且可视,如果是起点 那么不回溯父节点可视化
		if fatherNode != startNode {
			needChangeParent = false
			//1.尝试回溯获得根父节点
			if fatherNode.Parent.Parent != nil { //
				tmpRootFatherNode = fatherNode.Parent.Parent
			}
			//2.尝试判断新根节点连通性
			for tmpRootFatherNode != nil {
				if m.JudgeLineObstacleNew(fatherNode.X, fatherNode.Y, tmpRootFatherNode.X, tmpRootFatherNode.Y) {
					needChangeParent = true
					availableRootFatherNode = tmpRootFatherNode  //先更新可用根父节点为第2...n级
					tmpRootFatherNode = tmpRootFatherNode.Parent //尝试下1级根父节点 如果下1级不可以直连 那么availableRootFatherNode记录的就是最后一次可直连的根父
				} else {
					break
				}
			}
			//3.判断是否改变根父节点 不是Fix 因为已经不在open中了 不需要判断是否小于原来的F 因为当前新F一定 <= 原来的F,不需要Fix则无事发生
			if needChangeParent {
				tmpG = m.euclideanDistance(fatherNode.X, fatherNode.Y, availableRootFatherNode.X, availableRootFatherNode.Y)
				newG = availableRootFatherNode.G + tmpG
				newF = newG //newF没有启发函数
				//2.注意 g也要更新 根父节点的g + 本身到新根父节点的g
				fatherNode.G = newG
				//3.更新为更小的F
				fatherNode.F = newF
				//4.更新为新的根父节点
				fatherNode.Parent = availableRootFatherNode
			}
		}
		// 标记当前父节点处于关闭列表 加入到关闭列表 并且清除开启列表标记 --重要！！因为ResetMap的时候 是从openList和closeList中Reset的
		fatherNode.signInCloseList1_ClearOpenFlag1()
		m.closeList1 = append(m.closeList1, fatherNode)
		// 检查是否到达终点
		if fatherNode == endNode {
			m.oneWayRetracePath(startNode, fatherNode) //当前是反向路径 方法内部需要反转initPath
			return true
		}
		// 检查所有相邻节点
		for i = 0; i < 8; i++ {
			newX = fatherNode.X + eightDirSlice[i][0]
			newY = fatherNode.Y + eightDirSlice[i][1]
			// 检查邻居节点索引是否在地图内
			if !m.isMapNode(newX, newY) {
				//fmt.Println("", "newX:", newX, " newY:", newY)
				continue
			}

			neighbor = m.Nodes[newX][newY]

			// 如果邻居是墙 或者 邻居已经被当做父节点检索之后(已经放入close中了) 那么跳过
			if neighbor.IsWall() || neighbor.isInCloseList1() {
				//fmt.Println("", "neighbor.X:", neighbor.X, " neighbor.Y:", neighbor.Y)
				continue
			}

			//先判断邻居有没有拓展价值 有的话记录临时G
			if i >= 4 {
				//	{-1, 1},  //left up 右上邻居 //index 4
				//	{1, 1},   //right up 右下邻居 //index 5
				//	{1, -1},  //right down 左下邻居  //index 6
				//	{-1, -1}, //left down 左上邻居 //index 7
				switch i {
				case 4: //右上邻居 2边是墙则忽略 1边是墙1边是地图外则忽略
					//! N
					//C !
					//2都是墙 忽略
					judgeWallNodeFirst = m.Nodes[fatherNode.X][fatherNode.Y+1]
					judgeWallNodeSecond = m.Nodes[fatherNode.X-1][fatherNode.Y]
				case 5: //右下邻居
					//C !
					//! N
					//2都是墙 忽略
					judgeWallNodeFirst = m.Nodes[fatherNode.X][fatherNode.Y+1]
					judgeWallNodeSecond = m.Nodes[fatherNode.X+1][fatherNode.Y]
				case 6: //左下邻居
					//! C
					//N !
					//2都是墙 忽略
					judgeWallNodeFirst = m.Nodes[fatherNode.X][fatherNode.Y-1]
					judgeWallNodeSecond = m.Nodes[fatherNode.X+1][fatherNode.Y]
				case 7: //左上邻居
					//N !
					//! C
					//2都是墙 忽略
					judgeWallNodeFirst = m.Nodes[fatherNode.X][fatherNode.Y-1]
					judgeWallNodeSecond = m.Nodes[fatherNode.X-1][fatherNode.Y]
				}
				//judgeWall
				if judgeWallNodeFirst.IsWall() && judgeWallNodeSecond.IsWall() {
					continue
				}
				//
				tmpG = math.Sqrt2
			} else {
				tmpG = 1
			}
			//================不管在不在open 都要与根父作一次可视化检查 1级是当前父节点(上面判断过了 肯定可以与所有可用邻居直连) 2级是当前父节点的父节点(根父节点)
			//3.可视化检测结束 当前可用父可能是 fatherNode本身 或者 2级以上根父
			//newG = m.euclideanDistance(availableRootFatherNode.X, availableRootFatherNode.Y, neighbor.X, neighbor.Y)
			//newF = 可用根父节点本身的G(这里等于fatherNode的F) + newG
			newG = fatherNode.G + tmpG
			newF = newG
			//-----------------------------------------------------------邻居判断在不在open中
			//如果邻居已经存在OpenList(节点只会有3种情况 不在open也不在close 只在open 只在close) 如果当前计算出来的新F值小于 自己 当前已存在Open中的F值 那么替换为新的F 并且Fix维护堆性质
			if neighbor.isInOpenList1() { //邻居已经有自己的父节点
				//对于当前处于open中的邻居 都是新的父节点 与 自己本身的代价进行比较
				//2.用来与自己旧的F对比 如果比之前的小就替换 否则什么都不做直接跳过
				if newF < neighbor.F {
					//2.注意 g也要更新
					//2.更新新的g
					neighbor.G = newG
					//3.更新为更小的F , 没有启发函数 所以与newG相同
					neighbor.F = newF
					//4.更新为新的根父节点
					neighbor.Parent = fatherNode
					//4.Fix open,索引就是neighbor节点存储的openListIndex
					Fix(&m.openList1, neighbor.IndexInOpenList)
				}
			} else {
				//可视化检测结束 当前可用父可能是 fatherNode本身 或者 2级以上根父
				//1.对于当前不处于open中的邻居 邻居本身没有父节点 不需要比较旧的邻居代价 直接计算当前新的父或者根父节点代价存储 并添加进open
				//2.注意 g也要更新
				//neighbor.G = availableRootFatherNode.G + newG
				//2.更新新的 上面的迪杰g 等同于 newF
				neighbor.G = newF
				//3.更新为更小的F
				neighbor.F = newF
				//4.更新为新的根父节点
				neighbor.Parent = fatherNode
				//5.Push openList1
				Push(&m.openList1, neighbor)
				//6.记得标记
				neighbor.signInOpenList1()
				m.pushTimes++
			}
			//fmt.Println("", "neighbor.X:", neighbor.X, " neighbor.Y:", neighbor.Y)
			//打印openlist
			//for i := 0; i < len(m.openListFirst); i++ {
			//	fmt.Println("openListFirst", "openListFirst.X:", m.openListFirst[i].X, "openListFirst.Y:", m.openListFirst[i].Y)
			//}
			//times++
			neighbor.isOut = true
		}
	}
	fmt.Println("开启列表为空")
	return false
}

// --------------------------------------------------improved BiDirection ThetA PathFind
func (m *Map) impBTAPathFind(startX, startY, endX, endY int) bool {
	startNode := m.Nodes[startX][startY]
	endNode := m.Nodes[endX][endY]
	// 检查起点或终点是否为墙
	if startNode.IsWall() || endNode.IsWall() {
		fmt.Println("起点或终点是墙")
		return false
	}
	//times := 0
	m.startNode = startNode
	m.endNode = endNode
	startNode.F, startNode.G = 0, 0
	startNode.signInOpenList1()
	Push(&m.openList1, startNode) //openList1
	endNode.F, endNode.G = 0, 0
	endNode.signInOpenList1()
	Push(&m.openList2, endNode) //openList2
	m.pushTimes = 2
	// record last smaller F node
	var lastSmallerNodeReverseOpen1, lastSmallerNodeReverseOpen2 *Node
	// impBTA LOOP
	for {
		if len(m.openList1) == 0 || len(m.openList2) == 0 {
			break
		}
		//POP
		lastSmallerNodeReverseOpen1 = Pop(&m.openList1)
		lastSmallerNodeReverseOpen2 = Pop(&m.openList2)
		//sign close and append closeList
		lastSmallerNodeReverseOpen1.signInCloseList1_ClearOpenFlag1()    //sign close1
		lastSmallerNodeReverseOpen2.signInCloseList2_ClearOpenFlag2()    //sign close2
		m.closeList1 = append(m.closeList1, lastSmallerNodeReverseOpen1) //append close1
		m.closeList2 = append(m.closeList2, lastSmallerNodeReverseOpen2) //append close2
		//forward
		if m.doImpBTA(lastSmallerNodeReverseOpen1, lastSmallerNodeReverseOpen2, &m.openList1, true) {
			return true
		}
		//reverse
		if m.doImpBTA(lastSmallerNodeReverseOpen2, lastSmallerNodeReverseOpen1, &m.openList2, false) {
			return true
		}
		//m.popTimes
		m.popTimes += 2
	}
	fmt.Println("openList1 or openList2 length is 0")
	return false
}

func (m *Map) doImpBTA(fatherNode, lastSmallerNodeInReverseOpen *Node, openList *[]*Node, isForward bool) bool {
	// 标记当前父节点处于关闭列表 加入到关闭列表 并且清除开启列表标记 --重要！！因为ResetMap的时候 是从openList和closeList中Reset的
	//------------------------------------------------------1.tmp data---------------------------------------------
	//tmp data
	neighborX, neighborY := 0, 0
	var rootParentNode, neighbor, judgeWallNodeFirst, judgeWallNodeSecond *Node
	var newF, newG, newg float32 = 0, 0, 0
	openNeedChangeF := false
	//--------------------------------------------------2.set fatherNode --- root father Bresenham obstacle
	if fatherNode.hasParent() && fatherNode.Parent.hasParent() { //fatherNode起点是没有parent的 更别说祖父
		rootParentNode = fatherNode.Parent.Parent
	} else {
		rootParentNode = nil
	}
	for { //loop update rootParent
		if rootParentNode == nil {
			break
		}
		if m.JudgeLineObstacleNew(rootParentNode.X, rootParentNode.Y, fatherNode.X, fatherNode.Y) {
			openNeedChangeF = true
			fatherNode.Parent = rootParentNode     //update now rootParentNode
			rootParentNode = rootParentNode.Parent //update rootParentNode.Parent for rootParentNode
		} else {
			break
		}
	}
	//check rootParentNode change then update F = rootParentNode.G + rootParentNode.euclideanDistance(FatherNode) + H(FatherNode - offNode)
	if openNeedChangeF {
		//1.cal and save newF 但是不Fix 因为这是open出来的父节点 不需要Fix了
		m.changeOpenParentAndNewGF_ImpBTA(fatherNode, lastSmallerNodeInReverseOpen)
	}
	// get neighbor(8 dir)
	for i := 0; i < 8; i++ {
		neighborX = fatherNode.X + eightDirSlice[i][0]
		neighborY = fatherNode.Y + eightDirSlice[i][1]
		// ----------------------------------------------check neighbor work
		if !m.isMapNode(neighborX, neighborY) {
			//fmt.Println("", "neighborX:", neighborX, " neighborY:", neighborY)
			continue
		}
		neighbor = m.Nodes[neighborX][neighborY]
		//邻居是墙
		if neighbor.IsWall() {
			continue
		}
		//邻居处于自己的close中 要判断下当前寻路方向 则跳过该邻居
		if isForward {
			if neighbor.isInCloseList1() {
				continue
			}
		} else {
			if neighbor.isInCloseList2() {
				continue
			}
		}
		// ---------------------------------------------------------0.判断邻居联通
		//1.New 首先邻居要能通过才有意义 能通过才能判断邻居是不是已经处于对立open或者close,或者处于自己的open
		if i >= 4 {
			//	{-1, 1},  //left up 右上邻居 //index 4
			//	{1, 1},   //right up 右下邻居 //index 5
			//	{1, -1},  //right down 左下邻居  //index 6
			//	{-1, -1}, //left down 左上邻居 //index 7
			switch i {
			case 4: //右上邻居 2边是墙则忽略 1边是墙1边是地图外则忽略
				//! N
				//C !
				//2都是墙 忽略
				judgeWallNodeFirst = m.Nodes[fatherNode.X][fatherNode.Y+1]
				judgeWallNodeSecond = m.Nodes[fatherNode.X-1][fatherNode.Y]
			case 5: //右下邻居
				//C !
				//! N
				//2都是墙 忽略
				judgeWallNodeFirst = m.Nodes[fatherNode.X][fatherNode.Y+1]
				judgeWallNodeSecond = m.Nodes[fatherNode.X+1][fatherNode.Y]
			case 6: //左下邻居
				//! C
				//N !
				//2都是墙 忽略
				judgeWallNodeFirst = m.Nodes[fatherNode.X][fatherNode.Y-1]
				judgeWallNodeSecond = m.Nodes[fatherNode.X+1][fatherNode.Y]
			case 7: //左上邻居
				//N !
				//! C
				//2都是墙 忽略
				judgeWallNodeFirst = m.Nodes[fatherNode.X][fatherNode.Y-1]
				judgeWallNodeSecond = m.Nodes[fatherNode.X-1][fatherNode.Y]
			}
			//judgeWall offset 如果斜向邻居被障碍挡住 跳过该邻居
			if judgeWallNodeFirst.IsWall() && judgeWallNodeSecond.IsWall() {
				continue
			}
			//end:斜向邻居联通 并且不在对立open或close中
			newg = math.Sqrt2
		} else {
			//end:直向邻居联通 并且不在对立open或close中
			newg = 1
		}
		//----------------------------------------1.先判断在不在对立的open或者close中 在的话相对于找到了连接路径 结束寻路
		if isForward {
			if neighbor.isInOpenList2() || neighbor.isInCloseList2() {
				//1.forward road (start -> father)(need reverse)
				m.oneWayRetracePathForImpBTA(m.startNode, fatherNode, true)
				//2.reverse road (neighbor -> end)(dont need reverse)
				m.oneWayRetracePathForImpBTA(m.endNode, neighbor, false)
				return true
			}
		} else {
			if neighbor.isInOpenList1() || neighbor.isInCloseList1() {
				//1.forward road (start -> neighbor)(need reverse)
				m.oneWayRetracePathForImpBTA(m.startNode, neighbor, true)
				//2.reverse road (father -> end)(dont need reverse)
				m.oneWayRetracePathForImpBTA(m.endNode, fatherNode, false)
				return true
			}
		}
		//-----------------------------------------2.不在对立open或者close 那么判断邻居本身在不在自己方向的open中 这次不需要辨别方向了 因为上面已经判断过对立存在的情况
		//0.在自己的open中则重新计算一次newF 当前新的父类的g + 当前邻居到对立open点的H
		newG = fatherNode.G + newg                                               //计算新G
		newF = newG + m.calCost_H_ImpBTA(neighbor, lastSmallerNodeInReverseOpen) //计算新F
		if neighbor.isInOpenList1() || neighbor.isInOpenList2() {                //IsPOCWOC
			//1.如果比之前的F更小
			if newF < neighbor.F {
				//2.更新新的G
				neighbor.G = newG
				//3.更新为更小的F
				neighbor.F = newF
				//4.这么关键的一步怎么能忘了 更新为新的父节点
				neighbor.Parent = fatherNode
				//5.Fix open,索引就是neighbor节点存储的openListIndex
				Fix(openList, neighbor.IndexInOpenList)
			}
		} else { //邻居不处于open中 那么首次加入open
			//新的G 新的F push并且标记在open中
			neighbor.G = newG
			neighbor.F = newF
			Push(openList, neighbor)
			//5.新增 邻居赋值当前父节点 并 标记邻居有父节点
			neighbor.Parent = fatherNode
			neighbor.signHasParent()
			//6.标记邻居在open中
			if isForward {
				neighbor.signInOpenList1()
			} else {
				neighbor.signInOpenList2()
			}
			m.pushTimes++
			//for i := 0; i < len(m.openListFirst); i++ {
			//	fmt.Println("openListFirst", "openListFirst.X:", m.openListFirst[i].X, "openListFirst.Y:", m.openListFirst[i].Y)
			//}
			//times++
			neighbor.isOut = true
		}
	}
	// 这一次弹出open没有符合条件的结束点
	return false
}

// CalRectObstacleNums_UseSAT x1, y1, x2, y2不可能是相同点
func (m *Map) CalRectObstacleNums_UseSAT(x1, y1, x2, y2 int) int {
	//1.不管坐标点如何 始终能找到当前给定矩形区域的左上角和右下角 -- 不一定和x1, y1, x2, y2重合
	smallerX, biggerX := getSmallerAndBiggerInt(x1, x2)
	smallerY, biggerY := getSmallerAndBiggerInt(y1, y2)
	//1 0 0 1 0
	//0 1 0 0 1
	//0 0 1 1 0
	//1 0 0 0 1
	//0 1 0 1 0
	//2.看看矩形区域在哪里 4种情况
	if smallerX == 0 && smallerY == 0 {
		//左上点是原点 直接返回右下点的SAT
		return m.Nodes[biggerX][biggerY].ObstacleSAT
	} else if smallerX == 0 {
		//左上点在第0行 右下点SAT - 同右下行,左上列-1列的点SAT
		return m.Nodes[biggerX][biggerY].ObstacleSAT - m.Nodes[biggerX][smallerY-1].ObstacleSAT
	} else if smallerY == 0 {
		//左上点在第0列 右下点SAT - 同左上点行-1,右下列的点SAT
		return m.Nodes[biggerX][biggerY].ObstacleSAT - m.Nodes[smallerX-1][biggerY].ObstacleSAT
	}
	//不是以上三种情况 右下点SAT - 右上点的上1点SAT - 左下点的左1点SAT + 左上点的左上点SAT
	return m.Nodes[biggerX][biggerY].ObstacleSAT - m.Nodes[smallerX-1][biggerY].ObstacleSAT - m.Nodes[biggerX][smallerY-1].ObstacleSAT + m.Nodes[smallerX-1][smallerY-1].ObstacleSAT
}

func getSmallerAndBiggerInt(a, b int) (smallerInt, biggerInt int) {
	if a > b {
		biggerInt = a
		smallerInt = b
	} else {
		biggerInt = b
		smallerInt = a
	}
	return smallerInt, biggerInt
}

// node For Cal G,diagonalNode For cal H
func (m *Map) calCost_H_ImpBTA(node, diagonalNode *Node) float32 {
	//H0 =  (局部障碍数/全局障碍数) ->全局障碍为0时 H0等于0
	//H1 = (局部障碍最大尺寸/全地图最大边长度)
	//H new = H0 + H1
	var rightDownSAT = m.CalRectObstacleNums_UseSAT(0, 0, m.Rows-1, m.Cols-1) //全图障碍数量
	var maxObstacleSize = 0                                                   //局部最大障碍尺寸
	var Hbase, H0, H1 float32
	//0.2点欧几里得距离
	Hbase = m.euclideanDistance(node.X, node.Y, diagonalNode.X, diagonalNode.Y)
	//1.如果地图无障碍 那么H0为0
	if rightDownSAT == 0 {
		H0 = 0
	} else {
		H0 = float32(m.CalRectObstacleNums_UseSAT(node.X, node.Y, diagonalNode.X, diagonalNode.Y)) / float32(rightDownSAT) //记得先转为float再除 防止小数丢弃
	}
	//2.暂时先遍历局部计算障碍最大尺寸
	//不管坐标点如何 始终能找到当前给定矩形区域的左上角和右下角 -- 不一定和x1, y1, x2, y2重合 --遍历当前每个点 点是墙 比较记录的尺寸 更新大尺寸 遍历完得到局部最大尺寸墙
	maxObstacleSize = m.GetLineObstacleMaxSizeImpSpeed(node.X, node.Y, diagonalNode.X, diagonalNode.Y)
	//3.计算H1
	H1 = float32(maxObstacleSize) / float32(m.Rows) //记得先转为float再除 防止小数丢弃
	return Hbase + H0 + H1
}

// 该方法是open父节点使用改进可视性更新新的F 参数:当前open节点，对立弹出的open节点
func (m *Map) changeOpenParentAndNewGF_ImpBTA(node, diagonalNode *Node) {
	//存储当前node新的G值  = (当前open节点的父节点g + 当前open节点的父节点->当前open节点)
	node.G = node.Parent.G + m.euclideanDistance(node.Parent.X, node.Parent.Y, node.X, node.Y)
	//欧几里得计算新F = G(当前open节点的父节点g + 当前open节点的父节点->当前open节点) + h(欧几里得距离) + calH(当前open节点 -> 对立弹出的open节点 形成的矩形区域)
	node.F = node.G + m.calCost_H_ImpBTA(node, diagonalNode)
}

// --------------------------------------------------Dynamic Ray Cast PathFind

func (m *Map) dynamicRayCastPathFind(startX, startY, endX, endY int) bool {
	//计算必要信息
	startNode := m.Nodes[startX][startY]
	endNode := m.Nodes[endX][endY]
	m.startNode = startNode
	m.endNode = endNode
	m.popTimes = 1
	//
	startNode.G = 0
	startNode.F = 0
	startNode.signInOpenList1()      // 标记open
	m.calStartNodeMoveDir(startNode) // 计算起点移动方向(已处理)
	Push(&m.openList1, startNode)    // 修改：传递指针，无返回值赋值
	m.pushTimes++
	//用固定的根父节点 区分 不断改变的父节点
	var originalNode *Node
	//Loop 判断当前新的FatherNode
	for len(m.openList1) > 0 {
		//1.弹出 添加进closeList
		originalNode = Pop(&m.openList1)
		//fmt.Println(originalNode.X, " ", originalNode.Y)
		m.closeList1 = append(m.closeList1, originalNode)
		originalNode.signInCloseList1_ClearOpenFlag1()
		m.popTimes++
		//2.Lazy Ray cast --不会对起点操作
		//m.lazyRayCastNodeForAstar(originalNode, startNode)
		// 检查是否到达终点
		if originalNode == endNode {
			m.oneWayRetracePath(startNode, originalNode) //当前是反向路径 方法内部需要反转initPath
			return true
		}
		//3.检查获取新邻居
		m.getInterestedNeighbor_ForDynamicRayCast(originalNode, endNode)
	}
	//默认寻路失败
	fmt.Println("开启列表为空 路径查询失败 请排查！")
	return false
}

//备份旧方法。。。。。。。。。。。。。。。。。
// 获取当前节点视角上的 感兴趣的可以相连的墙角 注意起点这个东西要判断下!!!
//func (m *Map) getInterestedNeighbor_ForDynamicRayCast(originalNode, endNode *Node) {
//	//------------------------------------------找齐感兴趣墙角类别索引
//	//1.先看能不能与终点相连 能的话再查看终点处于open当中 不处于则添加进open退出
//	if canLink, firstObstacleNode := m.JudgeLineObstacleNew_GetOneEdgeObstacle(originalNode.X, originalNode.Y, endNode.X, endNode.Y); canLink {
//		if endNode.isInOpenList1() { //1.与旧的F比较 更小则替换并且Fix
//			m.tryFixNode_ForDynamicRay(originalNode, endNode)
//		} else { //2.不在open中则加入到open 在这里不会判断在close中 因为endNode弹出时已经结束寻路
//			m.addNodeToOpen_ForDynamicRay(originalNode, endNode)
//		}
//	} else { //2.不能与终点相连 firstObstacleNode是不为空的某个障碍物边界节点
//		var firstObstacleTypeIndex int
//		var interestedNode, otherInterestedNode *Node
//		//3.先判断第1个障碍物是否预存了边界信息
//		if firstObstacleNode.IIndex == -1 { //无预存障碍边界索引 计算出来
//			m.SetNewOneInterestedIndex_seedEdge(firstObstacleNode)
//		}
//		//4.记录最开始初始的障碍边界索引 标记为不重复障碍类别
//		firstObstacleTypeIndex = firstObstacleNode.IIndex
//		m.checkAndSetRepeatTemInterest_TypeIndex_BitMap(firstObstacleTypeIndex) //最开始初始类别是首位加入记录 肯定不重复
//		m.checkAndSetRepeatTemInterest_SingleNode_BitMap(originalNode)          //注意 父节点本身在此看作已判断过的感兴趣节点 防止邻居是同一点自己判断自己
//		//4.此时IIndex已预存 遍历当前IIndex所属的墙角 可以相连 则再查看在不在open中
//		for i := 0; i < len(m.Real_InterestNodeSlice[firstObstacleTypeIndex]); i++ {
//			//5.准备判断初始类别墙角可以相连
//			interestedNode = m.Real_InterestNodeSlice[firstObstacleTypeIndex][i]
//			//注意 并且由于父节点可能会被初始索引访问到(父节点本身也算感兴趣墙角的情况) 也可能是其他索引类别的感兴趣墙角(在遍历循环之前,已经被记录阻断了不会判断本身)
//			if interestedNode == originalNode {
//				continue
//			}
//			//6.判断初始类别墙角可以相连 墙角就是邻居 墙角FIX或Add
//			canLink, firstObstacleNode = m.JudgeLineObstacleNew_GetOneEdgeObstacle(originalNode.X, originalNode.Y, interestedNode.X, interestedNode.Y)
//			if canLink {
//				//6.记录并检查初始障碍类别可相连感兴趣墙角
//				if !m.checkAndSetRepeatTemInterest_SingleNode_BitMap(interestedNode) { //如果是不重复的感兴趣墙角 那么fix或者add
//					m.fixOrAddNode(originalNode, interestedNode)
//				}
//			} else { //6.与初始障碍类别中的某个墙角不可以相连 那么被某个障碍物挡住了 判断阻挡的新第1个障碍有没有分类
//				//6.不可以相连 那么也要记录该感兴趣墙角已判断
//				m.checkAndSetRepeatTemInterest_SingleNode_BitMap(interestedNode)
//				//7.新的障碍物边界逻辑
//				if firstObstacleNode.IIndex == -1 && firstObstacleNode.IIndex != firstObstacleTypeIndex { //障碍节点无预存障碍边界索引 且 不是初始障碍索引类别 才计算新的!
//					m.SetNewOneInterestedIndex_seedEdge(firstObstacleNode)
//				}
//				//7.检查并记录当前可能的新障碍索引类别 任何类别障碍索引都有可能 -- 所以只能检查不是原类别的索引 那么边判断边遍历里面的感兴趣墙角
//				if !m.checkAndSetRepeatTemInterest_TypeIndex_BitMap(firstObstacleNode.IIndex) {
//					//8.遍历里面的感兴趣墙角 --障碍类别索引肯定不重复 但是感兴趣墙角有可能属于多个类别,或者说1个感兴趣墙角有可能之前就处理过
//					//所以再次判断感兴趣墙角是否被处理过
//					for j := 0; j < len(m.Real_InterestNodeSlice[firstObstacleNode.IIndex]); j++ {
//						otherInterestedNode = m.Real_InterestNodeSlice[firstObstacleNode.IIndex][j]
//						if !m.checkAndSetRepeatTemInterest_SingleNode_BitMap(otherInterestedNode) { //如果本次没有判断过的感兴趣墙角 那么判断相连
//							if canLink, _ = m.JudgeLineObstacleNew_GetOneEdgeObstacle(originalNode.X, originalNode.Y, otherInterestedNode.X, otherInterestedNode.Y); canLink {
//								m.fixOrAddNode(originalNode, otherInterestedNode) //相连则Fix或者Add
//							}
//						}
//					}
//				}
//			}
//		}
//	}
//	//End:重置一些数据
//	m.clearTemInterest_TypeIndex_BitMap()
//	m.clearTemInterestSingleNodeBitMap()
//}
// 获取当前节点视角上的 感兴趣的可以相连的墙角 注意起点这个东西要判断下!!!

// m.temInterest_TypeIndex_Slice                      []int
// m.temInterest_TypeIndex_CheckRepeatBitMap          []byte
// m.temInterest_SingleIndexNode_Slice                []int
// m.temInterest_SingleNode_CheckRepeatBitMap         []byte
func (m *Map) getInterestedNeighbor_ForDynamicRayCast(originalNode, endNode *Node) {
	//------------------------------------------找齐感兴趣墙角类别索引
	//1.先看能不能与终点相连 能的话再查看终点处于open当中 不处于则添加进open退出
	if canLink, firstObstacleNode := m.JudgeLineObstacleNew_GetOneEdgeObstacle_UseBitMap(originalNode.X, originalNode.Y, endNode.X, endNode.Y); canLink {
		if endNode.isInOpenList1() { //1.与旧的F比较 更小则替换并且Fix
			m.tryFixNode_ForDynamicRay(originalNode, endNode)
		} else { //2.不在open中则加入到open 在这里不会判断在close中 因为endNode弹出时已经结束寻路
			m.addNodeToOpen_ForDynamicRay(originalNode, endNode)
		}
	} else { //2.不能与终点相连 firstObstacleNode是不为空的某个障碍物边界节点
		var firstObstacleTypeIndex int
		var interestedNode, otherInterestedNode *Node
		//3.先判断第1个障碍物是否预存了边界信息
		if firstObstacleNode.IIndex == -1 { //无预存障碍边界索引 计算出来
			m.SetNewOneInterestedIndex_seedEdge(firstObstacleNode)
		}
		//4.记录最开始初始的障碍边界索引 标记为不重复障碍类别
		firstObstacleTypeIndex = firstObstacleNode.IIndex
		m.SetSame_BitMap(firstObstacleTypeIndex, &m.temInterest_TypeIndex_Slice, m.temInterest_TypeIndex_CheckRepeatBitMap)          //最开始初始类别是首位加入记录 肯定不重复
		m.SetSame_BitMap(originalNode.singleIndex, &m.temInterest_SingleIndexNode_Slice, m.temInterest_SingleNode_CheckRepeatBitMap) //注意 父节点本身在此看作已判断过的感兴趣节点 防止邻居是同一点自己判断自己
		//4.此时IIndex已预存 遍历当前IIndex所属的墙角 可以相连 则再查看在不在open中
		for i := 0; i < len(m.Real_InterestNodeSlice[firstObstacleTypeIndex]); i++ {
			//5.准备判断初始类别墙角可以相连
			interestedNode = m.Real_InterestNodeSlice[firstObstacleTypeIndex][i]
			//阻断：已经被记忆的感兴趣邻居不会再判断 感兴趣邻居有可能是父节点本身
			if m.checkSame_BitMap(interestedNode.singleIndex, m.temInterest_SingleNode_CheckRepeatBitMap) {
				continue
			}
			//不管相连与否 都要记忆该邻居已被判断过
			m.SetSame_BitMap(interestedNode.singleIndex, &m.temInterest_SingleIndexNode_Slice, m.temInterest_SingleNode_CheckRepeatBitMap)
			//6.判断初始类别感兴趣墙角是否可以相连 墙角就是邻居 墙角FIX或Add
			canLink, firstObstacleNode = m.JudgeLineObstacleNew_GetOneEdgeObstacle_UseBitMap(originalNode.X, originalNode.Y, interestedNode.X, interestedNode.Y)
			if canLink {
				//6.记录并检查初始障碍类别可相连感兴趣墙角
				//如果是不重复的感兴趣墙角 那么fix或者add 并且记录该感兴趣墙角已经访问过
				m.fixOrAddNode(originalNode, interestedNode)
			} else { //6.与初始障碍类别中的某个墙角不可以相连 那么被某个障碍物挡住了
				//7.判断阻挡的新第1个障碍有没有分类
				if firstObstacleNode.IIndex == -1 && firstObstacleNode.IIndex != firstObstacleTypeIndex { //障碍节点无预存障碍边界索引 且 不是初始障碍索引类别 才计算新的!
					m.SetNewOneInterestedIndex_seedEdge(firstObstacleNode)
				}
				//8.阻断：当前新障碍的IIndex是否已被记忆过
				if m.checkSame_BitMap(firstObstacleNode.IIndex, m.temInterest_TypeIndex_CheckRepeatBitMap) {
					continue
				}
				//9.检查并记录当前可能的新障碍索引类别 任何类别障碍索引都有可能 -- 所以只能检查不是原类别的索引 那么边判断边遍历里面的感兴趣墙角
				m.SetSame_BitMap(firstObstacleNode.IIndex, &m.temInterest_TypeIndex_Slice, m.temInterest_TypeIndex_CheckRepeatBitMap)
				//10.遍历里面的感兴趣墙角 --障碍类别索引肯定不重复 但是感兴趣墙角有可能属于多个类别,或者说1个感兴趣墙角有可能之前就处理过
				//所以再次判断感兴趣墙角是否被处理过
				for j := 0; j < len(m.Real_InterestNodeSlice[firstObstacleNode.IIndex]); j++ {
					otherInterestedNode = m.Real_InterestNodeSlice[firstObstacleNode.IIndex][j]
					//11.如果本次感兴趣墙角没有记忆过 那么记忆
					if m.checkSame_BitMap(otherInterestedNode.singleIndex, m.temInterest_SingleNode_CheckRepeatBitMap) {
						continue
					}
					//那么记忆
					m.SetSame_BitMap(otherInterestedNode.singleIndex, &m.temInterest_SingleIndexNode_Slice, m.temInterest_SingleNode_CheckRepeatBitMap)
					//并判断相连
					if canLink, _ = m.JudgeLineObstacleNew_GetOneEdgeObstacle_UseBitMap(originalNode.X, originalNode.Y, otherInterestedNode.X, otherInterestedNode.Y); canLink {
						m.fixOrAddNode(originalNode, otherInterestedNode) //相连则Fix或者Add
					}
				}
			}
		}
	}
	//End:重置一些数据
	m.clear_BitMap(&m.temInterest_TypeIndex_Slice, m.temInterest_TypeIndex_CheckRepeatBitMap)
	m.clear_BitMap(&m.temInterest_SingleIndexNode_Slice, m.temInterest_SingleNode_CheckRepeatBitMap)
}

// SetNewOneInterestedIndex_seedEdge --------------------------------------------------------------------边探索障碍种子 边判断非障碍感兴趣墙角
func (m *Map) SetNewOneInterestedIndex_seedEdge(firstObstacleNode *Node) int {
	//就找当前障碍所围成的没有记录(计算)过的新的障碍边界和感兴趣墙角
	obstacleEdgeIndex := m.ObstacleEdgeIndex
	m.obstacleEdgeSeedNode_Pop_Slice = append(m.obstacleEdgeSeedNode_Pop_Slice, firstObstacleNode) //添加第1个障碍
	var seedNode, seedNeighborNode, roadNode *Node = nil, nil, nil
	var nowLen, seedX, seedY, judgeSeedX, judgeSeedY, roadX, roadY = 1, -1, -1, -1, -1, -1, -1
	var needAddThisNewSeed = false
	//首先标记当前节点已经被添加过种子 因为它肯定是障碍边界 并且记录要删除标记列表 并且当前设置当前障碍边界索引
	m.Real_ObstacleEdgeSlice[obstacleEdgeIndex] = append(m.Real_ObstacleEdgeSlice[obstacleEdgeIndex], firstObstacleNode)                                        //记录障碍边界分类 --更新障碍位置要重置障碍边界记录的分类索引
	m.checkAndSetSame_BitMap(firstObstacleNode.singleIndex, &m.temMemoryObstacle_SeedJudge_SingleIndex_Slice, m.temMemoryObstacle_SeedJudge_SingleIndex_BitMap) //临时种子记忆记录
	//1.对障碍源点本身进行判断 只判断感兴趣节点 不找新的障碍边界 只找感兴趣墙角
	seedX, seedY = firstObstacleNode.X, firstObstacleNode.Y
	for i := 0; i < 8; i++ {
		//感兴趣墙角是否越界
		if !firstObstacleNode.checkNeighbor_IsMapNode(i) {
			continue
		}
		//感兴趣墙角不越界
		roadX, roadY = seedX+eightDirSlice[i][0], seedY+eightDirSlice[i][1]
		roadNode = m.Nodes[roadX][roadY]
		if roadNode.IsRoad() { //非障碍邻居 才能进行感兴趣墙角判断
			//判断墙角有没有检查过 没有检查过才检查
			if !m.checkAndSetSame_BitMap(roadNode.singleIndex, &m.temMemoryRoad_InterestedJudge_SingleIndex_Slice, m.temMemoryRoad_InterestedJudge_SingleIndex_BitMap) {
				if m.checkRoadIsInterestedNode(roadNode) { //实际判断是感兴趣墙角 有可能同时属于多个障碍
					m.Real_InterestNodeSlice[obstacleEdgeIndex] = append(m.Real_InterestNodeSlice[obstacleEdgeIndex], roadNode) //记忆有效墙角
					//m.testPrint_InterestNodeMap[roadNode] = obstacleEdgeIndex                                                   //记得删掉！！！！！！！！只是打印用
				}
			}
		}
	}
	//2.源点障碍种子扫描边界方法 边扫描障碍边界 边判断非障碍感兴趣节点--比分开判断要快一倍 因为非障碍感兴趣节点就是障碍边界邻居
	for len(m.obstacleEdgeSeedNode_Pop_Slice) > 0 {
		//1.每次取出最后一位当做障碍种子判断 更新切片长度
		nowLen = len(m.obstacleEdgeSeedNode_Pop_Slice)
		seedNode = m.obstacleEdgeSeedNode_Pop_Slice[nowLen-1]
		m.obstacleEdgeSeedNode_Pop_Slice = m.obstacleEdgeSeedNode_Pop_Slice[:nowLen-1]
		seedX, seedY = seedNode.X, seedNode.Y
		//2.障碍种子本身赋值IIndex
		seedNode.IIndex = obstacleEdgeIndex
		//2.对障碍种子8邻居判断 如果是障碍并且没有被探索过 那么才进入更深层次的判断
		for i := 0; i < 8; i++ {
			//障碍种子是否越界
			if !seedNode.checkNeighbor_IsMapNode(i) {
				continue
			}
			//不越界 获取相邻新的障碍边界
			judgeSeedX, judgeSeedY = seedX+eightDirSlice[i][0], seedY+eightDirSlice[i][1]
			seedNeighborNode = m.Nodes[judgeSeedX][judgeSeedY]
			//没有被种子探索加入临时探索切片探索过
			if m.checkAndSetSame_BitMap(seedNeighborNode.singleIndex, &m.temMemoryObstacle_SeedJudge_SingleIndex_Slice, m.temMemoryObstacle_SeedJudge_SingleIndex_BitMap) {
				continue
			}
			//3.如果障碍种子的邻居是障碍 那么将它当做种子加入临时探索切片 并且记忆
			if seedNeighborNode.IsWall() { //障碍种子邻居
				//4.边找新障碍种子 边找感兴趣墙角
				for k := 0; k < 8; k++ {
					//感兴趣墙角是否越界
					if !seedNeighborNode.checkNeighbor_IsMapNode(k) {
						continue
					}
					//感兴趣墙角不越界
					roadX, roadY = judgeSeedX+eightDirSlice[k][0], judgeSeedY+eightDirSlice[k][1]
					roadNode = m.Nodes[roadX][roadY]
					if roadNode.IsRoad() { //非障碍邻居
						needAddThisNewSeed = true
						//判断有没有检查过 没有检查过才继续
						if !m.checkAndSetSame_BitMap(roadNode.singleIndex, &m.temMemoryRoad_InterestedJudge_SingleIndex_Slice, m.temMemoryRoad_InterestedJudge_SingleIndex_BitMap) {
							if m.checkRoadIsInterestedNode(roadNode) { //实际判断是感兴趣墙角 有可能同时属于多个障碍
								m.Real_InterestNodeSlice[obstacleEdgeIndex] = append(m.Real_InterestNodeSlice[obstacleEdgeIndex], roadNode) //记忆有效墙角
								//m.testPrint_InterestNodeMap[roadNode] = obstacleEdgeIndex                                                   //记得删掉！！！！！！！！只是打印用
							}
						}
					}
				}
				//障碍种子邻居是否被当做障碍种子边界继续判断
				if needAddThisNewSeed {
					m.obstacleEdgeSeedNode_Pop_Slice = append(m.obstacleEdgeSeedNode_Pop_Slice, seedNeighborNode)                       //临时拓展种子集合 					//双向节点索引记录
					m.Real_ObstacleEdgeSlice[obstacleEdgeIndex] = append(m.Real_ObstacleEdgeSlice[obstacleEdgeIndex], seedNeighborNode) //记录障碍边界分类 --更新障碍位置要重置障碍边界记录的分类索引
				}
			}
		}
	}
	//End:m.ObstacleEdgeIndex++
	m.ObstacleEdgeIndex++
	//重置标记切片 重置障碍种子切片
	m.obstacleEdgeSeedNode_Pop_Slice = m.obstacleEdgeSeedNode_Pop_Slice[:0]
	m.clear_BitMap(&m.temMemoryObstacle_SeedJudge_SingleIndex_Slice, m.temMemoryObstacle_SeedJudge_SingleIndex_BitMap)
	m.clear_BitMap(&m.temMemoryRoad_InterestedJudge_SingleIndex_Slice, m.temMemoryRoad_InterestedJudge_SingleIndex_BitMap)
	//返回本次计算好的障碍感兴趣墙角类别索引
	return obstacleEdgeIndex
}

// 记得处理越界问题 先判断斜向点是否越界 不越界进一步判断
func (m *Map) checkRoadIsInterestedNode(roadNode *Node) bool {
	//先看斜向是否越界
	//以右上 右下 左下 左上为基准点
	for i := 4; i < 8; i++ {
		//先判断4斜向邻居是否越界
		if !roadNode.checkNeighbor_IsMapNode(i) {
			continue
		}
		//斜向不越界 那么斜向水平垂直邻居肯定不越界
		//非障碍的邻居 判断墙角:有至少1个对角不可走 阻挡对对角的相邻2格可走 --也就是4个方向对角 如果有1个满足 那么就是墙角
		if roadNode.checkNeighbor_IsWall(i) {
			//获取斜向相邻水平垂直2点(矩形地图必定不越界)
			//如果都符合不是墙 那么是有效墙角 停止并返回true 否侧继续下一个斜向判断
			if !roadNode.checkNeighbor_IsWall(eightDir_HV_Neighbor_Slice[i-4][0]) && !roadNode.checkNeighbor_IsWall(eightDir_HV_Neighbor_Slice[i-4][1]) {
				return true
			}
		}
	}
	return false
}

// 清除临时位图不重复的感兴趣记录
func (m *Map) clear_BitMap(slice *[]int, bitMap []byte) {
	//temInterestNodeSingleIndexSlice才是不重复的位图索引
	for i := 0; i < len(*slice); i++ {
		bitMap[(*slice)[i]/8] = 0
	}
	//end：清空不重复索引
	*slice = (*slice)[:0]
}

// ---------------------------------------新写 通用只检索不重复节点或索引方法
func (m *Map) checkSame_BitMap(id int, bitMap []byte) bool {
	bitIndex, bitRightDownOffset := id/8, id%8 //相对于 n >>3  , n & 7(0b00000111)
	//println(id, "\t", bitIndex)
	return (128 >> bitRightDownOffset & bitMap[bitIndex]) != 0 //不等于0代表有记录 属于同一个--也就是有记忆
}

// 边判断临时位图不重复边记录 把位图每1位当做0到正整数不重复顺序排列 01234567,8910....
func (m *Map) SetSame_BitMap(id int, slice *[]int, bitMap []byte) {
	bitIndex, bitRightDownOffset := id/8, id%8               //相对于 n >>3  , n & 7(0b00000111)
	if (128 >> bitRightDownOffset & bitMap[bitIndex]) == 0 { //等于0代表之前没有记忆 那么是不重复的点 需要添加记录
		*slice = append(*slice, id)
		bitMap[bitIndex] |= 128 >> bitRightDownOffset
	}
}

// 边判断临时位图不重复边记录 把位图每1位当做0到正整数不重复顺序排列 01234567,8910....
func (m *Map) checkAndSetSame_BitMap(id int, slice *[]int, bitMap []byte) bool {
	bitIndex, bitRightDownOffset := id/8, id%8               //相对于 n >>3  , n & 7(0b00000111)
	if (128 >> bitRightDownOffset & bitMap[bitIndex]) == 0 { //等于0代表之前没有记忆 那么是不重复的点 需要添加记录
		*slice = append(*slice, id)
		bitMap[bitIndex] |= 128 >> bitRightDownOffset
		return false
	}
	return true
}

// -------------------------------------open相关
func (m *Map) fixOrAddNode(originalNode, neighborNode *Node) {
	if neighborNode.isInOpenList1() { //1.在open中与旧的F比较 更小则替换并且Fix
		m.tryFixNode_ForDynamicRay(originalNode, neighborNode)
	} else if neighborNode.isInCloseList1() { //2.在close中 跳过该墙角
		return
	} else { //3.不在open和close中 加入open
		m.addNodeToOpen_ForDynamicRay(originalNode, neighborNode)
	}
}

// 明确邻居在open中 可能需要fix
func (m *Map) tryFixNode_ForDynamicRay(fatherNode, neighbor *Node) {
	var tmpRootFatherNode, availableRootFatherNode *Node = nil, fatherNode
	//1.尝试回溯获得根父节点
	if fatherNode != m.startNode {
		tmpRootFatherNode = fatherNode.Parent
	}
	//2.尝试判断新根节点连通性
	for tmpRootFatherNode != nil {
		if m.JudgeLineObstacleNew(neighbor.X, neighbor.Y, tmpRootFatherNode.X, tmpRootFatherNode.Y) {
			availableRootFatherNode = tmpRootFatherNode //先更新可用根父节点为第2...n级
			//tmpRootFatherNode = tmpRootFatherNode.Parent //尝试下1级根父节点 如果下1级不可以直连 那么availableRootFatherNode记录的就是最后一次可直连的根父
		}
		//就算不能与当前根父节点相连 也要一直回溯 直到没有根父节点
		tmpRootFatherNode = tmpRootFatherNode.Parent
	}
	//3.判断当前新F <= 原来的F,需要Fix
	newG := availableRootFatherNode.G + m.euclideanDistance(neighbor.X, neighbor.Y, availableRootFatherNode.X, availableRootFatherNode.Y)
	newF := newG //newF没有启发函数
	if newF < neighbor.F {
		//2.注意 g也要更新 根父节点的g + 本身到新根父节点的g
		neighbor.G = newG
		//3.更新为更小的F
		neighbor.F = newF
		//4.更新为新的根父节点
		neighbor.Parent = availableRootFatherNode
		//Fix
		Fix(&m.openList1, neighbor.IndexInOpenList)
		m.FixTimes++
	}

}

// 明确邻居需要添加进open中
func (m *Map) addNodeToOpen_ForDynamicRay(fatherNode, neighbor *Node) {
	var tmpRootFatherNode, availableRootFatherNode *Node = nil, fatherNode
	//1.尝试回溯获得根父节点
	if fatherNode != m.startNode {
		tmpRootFatherNode = fatherNode.Parent
	}
	//2.尝试判断新根节点连通性
	for tmpRootFatherNode != nil {
		if m.JudgeLineObstacleNew(neighbor.X, neighbor.Y, tmpRootFatherNode.X, tmpRootFatherNode.Y) {
			availableRootFatherNode = tmpRootFatherNode //先更新可用根父节点为第2...n级
			//tmpRootFatherNode = tmpRootFatherNode.Parent //尝试下1级根父节点 如果下1级不可以直连 那么availableRootFatherNode记录的就是最后一次可直连的根父
		}
		//就算不能与当前根父节点相连 也要一直回溯 直到没有根父节点
		tmpRootFatherNode = tmpRootFatherNode.Parent
	}
	//3.判断是否改变根父节点 不是Fix 因为已经不在open中了 不需要判断是否小于原来的F 因为当前新F一定 <= 原来的F,不需要Fix则无事发生
	newG := availableRootFatherNode.G + m.euclideanDistance(neighbor.X, neighbor.Y, availableRootFatherNode.X, availableRootFatherNode.Y)
	newF := newG //newF没有启发函数
	//2.注意 g也要更新 根父节点的g + 本身到新根父节点的g
	neighbor.G = newG
	//3.更新为更小的F
	neighbor.F = newF
	//4.更新为新的根父节点
	neighbor.Parent = availableRootFatherNode
	//push
	neighbor.signInOpenList1()
	Push(&m.openList1, neighbor)
	m.pushTimes++
}

// --------------------------------------------------打印

// PrintInitNode 打印路径
func (m *Map) PrintInitNode() {
	fmt.Println("初始路径为:")
	//打印初始路径点
	for i := 0; i < len(m.InitPath); i++ {
		fmt.Println("InitPath-序号", i, "：", "X:", m.InitPath[i].X, "\t", "Y:", m.InitPath[i].Y)
	}
}

// PrintJudgeMap 打印Bresenham像素点位
func (m *Map) PrintJudgeMap(judgeNodeSlice []*Node) {
	//
	fmt.Println("\nPrintJudgeMap：")
	find := false
	for i := 0; i < len(m.Nodes); i++ {
		for j := 0; j < len(m.Nodes[i]); j++ {
			for _, judgeNode := range judgeNodeSlice {
				find = m.Nodes[i][j] == judgeNode
				if find {
					break
				}
			}
			//
			if find {
				fmt.Print("!")
			} else {
				fmt.Print(".")
			}
		}
		fmt.Println()
	}
	fmt.Println()
}

// PrintOutMap 打印地图上已访问的节点 make([]string, 0, m.Rows) []
func (m *Map) PrintOutMap() {
	//
	fmt.Println("\nPrintOutMap：")
	for i := 0; i < len(m.Nodes); i++ {
		for j := 0; j < len(m.Nodes[i]); j++ {
			if m.Nodes[i][j].IsWall() {
				fmt.Print("!")
			} else {
				//
				if m.Nodes[i][j].isOut {
					fmt.Print("j")
				} else {
					fmt.Print(".")
				}
			}
		}
		fmt.Println()
	}
	fmt.Println()
}

func (m *Map) PrintMap() {
	//
	fmt.Println("\nPrintMap：")
	for i := 0; i < len(m.Nodes); i++ {
		for j := 0; j < len(m.Nodes[i]); j++ {
			if m.Nodes[i][j].IsWall() {
				fmt.Print("!")
			} else {
				fmt.Print(".")
			}
		}
		fmt.Println()
	}
	fmt.Println()
}

// PrintInitPathMap 可视化地图和路径 make([]string, 0, m.Rows) []
func (m *Map) PrintInitPathMap() {
	//
	fmt.Println("\nPrintInitPathMap：")
	findFlag := false
	for i := 0; i < len(m.Nodes); i++ {
		for j := 0; j < len(m.Nodes[i]); j++ {
			if m.Nodes[i][j].IsWall() {
				fmt.Print("!")
			} else {
				for k := 0; k < len(m.InitPath); k++ {
					if m.InitPath[k] == m.Nodes[i][j] {
						findFlag = true
						break
					}
				}
				//
				if findFlag {
					fmt.Print("*")
				} else {
					fmt.Print(".")
				}
				findFlag = false
			}
		}
		fmt.Println()
	}
	fmt.Println()
}

// PrintInitPathMapAfterStep1Map 双向弗洛伊德 可视化地图和路径 make([]string, 0, m.Rows) []
func (m *Map) PrintInitPathMapAfterStep1Map() {
	//
	fmt.Println("\nPrintInitPathMapAfterStep1 双向弗洛伊德：")
	findFlag := false
	for i := 0; i < len(m.Nodes); i++ {
		for j := 0; j < len(m.Nodes[i]); j++ {
			if m.Nodes[i][j].IsWall() {
				fmt.Print("!")
			} else {
				for k := 0; k < len(m.InitPath); k++ {
					if m.InitPath[k] == m.Nodes[i][j] {
						findFlag = true
						break
					}
				}
				//
				if findFlag {
					fmt.Print("*")
				} else {
					fmt.Print(".")
				}
				findFlag = false
			}
		}
		fmt.Println()
	}
	fmt.Println()
}

// printResultMap 可视化地图和优化后的路径
func (m *Map) printResultMap() {
	fmt.Println("\nprintResultMap ：")
	findFlag := false
	for i := 0; i < len(m.Nodes); i++ {
		for j := 0; j < len(m.Nodes[i]); j++ {
			if m.Nodes[i][j].IsWall() {
				fmt.Print("!")
			} else {
				// 检查是否为优化后路径上的点
				for k := 0; k < len(m.ResultNodeList); k++ {
					if m.ResultNodeList[k] == m.Nodes[i][j] {
						findFlag = true
						break
					}
				}
				//
				if findFlag {
					fmt.Print("*") // 用'*'表示优化后的路径
				} else {
					fmt.Print(".")
				}
				findFlag = false
			}
		}
		fmt.Println()
	}
}

func (m *Map) printResult() {
	fmt.Println("当前A星后处理最终路径:")
	for i := 0; i < len(m.ResultNodeList); i++ {
		fmt.Println("ResultNodeList-序号", i, "：", "X:", m.ResultNodeList[i].X, "\t", "Y:", m.ResultNodeList[i].Y)
	}
}

// PrintTestJudgeObstacleLineMap 输出直线障碍判断经过的判断点
func (m *Map) PrintTestJudgeObstacleLineMap(judgeNodeSlice []*Node) {
	fmt.Println("\n输出直线障碍判断经过的判断点：")
	findFlag := false
	for i := 0; i < len(m.Nodes); i++ {
		for j := 0; j < len(m.Nodes[i]); j++ {
			if m.Nodes[i][j].IsWall() {
				fmt.Print("!")
			} else {
				// 检查是否找到目标切片点
				for k := 0; k < len(judgeNodeSlice); k++ {
					if judgeNodeSlice[k] == m.Nodes[i][j] {
						findFlag = true
						break
					}
				}
				//
				if findFlag {
					fmt.Print("#") // #代表经过的判断点(像素点)
				} else {
					fmt.Print(".")
				}
				findFlag = false
			}
		}
		fmt.Println()
	}
}

func (m *Map) Print_DynamicRayMemory_Obstacle_IIndexMap() {
	//
	fmt.Println("\nPrintMap：")
	for i := 0; i < len(m.Nodes); i++ {
		for j := 0; j < len(m.Nodes[i]); j++ {
			if m.Nodes[i][j].IIndex != -1 {
				fmt.Print(m.Nodes[i][j].IIndex)
			} else {
				fmt.Print(".")
			}
		}
		fmt.Println()
	}
	fmt.Println()
}

func (m *Map) Print_DynamicRayMemory_Road_InterestNode_Map() {
	//
	fmt.Println("\nPrintMap：")
	for i := 0; i < len(m.Nodes); i++ {
		for j := 0; j < len(m.Nodes[i]); j++ {
			if _, ok := m.testPrint_InterestNodeMap[m.Nodes[i][j]]; ok {
				fmt.Print("*")
			} else {
				fmt.Print(".")
			}
		}
		fmt.Println()
	}
	fmt.Println()
}

// -------------------------------------------------- 可能用到的式子
// 对角线距离：计算两个节点之间的距离 (对角线距离 切比雪夫最准确)
func (m *Map) diagonalDistance(startX, startY, endX, endY int) float32 {
	dx := math.Abs(float64(startX - endX))
	dy := math.Abs(float64(startY - endY))
	return float32(dx + dy + (math.Sqrt2-2)*math.Min(dx, dy))
	//return float32(math.Sqrt(dx*dx + dy*dy))
	//if dx == 0 || dy == 0 {
	//	return float32(dx + dy)
	//} else {
	//	return float32(dx + dy - (2-math.Sqrt2)*math.Min(dx, dy))
	//}
}

// 欧几里得
func (m *Map) euclideanDistance(startX, startY, endX, endY int) float32 {
	dx := math.Abs(float64(startX - endX))
	dy := math.Abs(float64(startY - endY))
	return float32(math.Sqrt(dx*dx + dy*dy))

}

func (m *Map) CalculateCross(neighbor *Node) float32 {
	// 计算 dx1, dy1（当前节点到终点的距离）
	dx1 := math.Abs(float64(neighbor.X - m.endNode.X))
	dy1 := math.Abs(float64(neighbor.Y - m.endNode.Y))

	// 计算 dx2, dy2（起点到终点的距离）
	dx2 := math.Abs(float64(m.startNode.X - m.endNode.X))
	dy2 := math.Abs(float64(m.startNode.Y - m.endNode.Y))

	// 计算叉积 cross
	//cross := math.Abs(dx1*dy2 - dx2*dy1)

	return float32(math.Abs(dx1*dy2 - dx2*dy1))
}

// 新增：整数绝对值函数（避免math.Abs的类型转换开销）
func absInt(x int) int {
	if x < 0 {
		return -x
	}
	return x
}

// 回溯路径:单方向通用 A星结束方法
func (m *Map) oneWayRetracePath(offNode, nowNode *Node) {
	//
	currentNode := nowNode
	// 遍历找父亲节点 直到自己就是结束点(起点或终点)为止
	for currentNode != offNode {
		m.InitPath = append(m.InitPath, currentNode)
		currentNode = currentNode.Parent
		//fmt.Println("死循环currentNode：", currentNode.X, " ", currentNode.Y)
	}
	m.InitPath = append(m.InitPath, offNode)
	// 反转路径
	reverserSlice(&m.InitPath)
}

func (m *Map) oneWayRetracePathForImpBTA(offNode, nowNode *Node, needReverse bool) {
	//
	currentNode := nowNode
	// 遍历找父亲节点 直到自己就是结束点(起点或终点)为止
	for currentNode != offNode {
		m.InitPath = append(m.InitPath, currentNode)
		currentNode = currentNode.Parent
	}
	m.InitPath = append(m.InitPath, offNode)
	if needReverse {
		// 反转路径
		reverserSlice(&m.InitPath)
	}
}

func newCross(firstNode, secondNode, thirdNode *Node) int {
	dx1 := secondNode.X - firstNode.X
	dy1 := secondNode.Y - firstNode.Y
	dx2 := thirdNode.X - firstNode.X
	dy2 := thirdNode.Y - firstNode.Y
	return dx1*dy2 - dy1*dx2
}

func isSameLine(firstNode, secondNode, thirdNode *Node) bool {
	return newCross(firstNode, secondNode, thirdNode) == 0
}

// -------------------------------------------------- 2点连线相关
// -------------------------------------------------- 提前计算一个相同地图大小的 左上角(0,0) 与 右上角(0,maxY) -> 到 任意点(n,n) 连线经过的点的嵌套切片坐标记录 正反都需要 一共4个
var zeroLinkAny_AllNode [][]int            //存储经过点整数坐标xy顺序排列 --也就是两整数为1点坐标
var zeroLinkAny_EdgeStartXIndex [][]int    //存储擦边点索引 --也就是AllNode内数组的某个擦边索引
var anyLinkZero_AllNode [][]int            //同上
var anyLinkZero_EdgeStartXIndex [][]int    //同上
var rightUpLinkAny_AllNode [][]int         //同上
var rightUpLinkAny_EdgeStartXIndex [][]int //同上
var anyLinkRightUp_AllNode [][]int         //同上
var anyLinkRightUp_EdgeStartXIndex [][]int //同上

func (m *Map) calLineObstacle_PreLoad() {
	//设计成所有实例共享 这个方法在每次调用时会清空重新make
	zeroLinkAny_AllNode = make([][]int, 0, m.Rows*m.Cols)
	zeroLinkAny_EdgeStartXIndex = make([][]int, 0, m.Rows*m.Cols)
	anyLinkZero_AllNode = make([][]int, 0, m.Rows*m.Cols)
	anyLinkZero_EdgeStartXIndex = make([][]int, 0, m.Rows*m.Cols)
	rightUpLinkAny_AllNode = make([][]int, 0, m.Rows*m.Cols)
	rightUpLinkAny_EdgeStartXIndex = make([][]int, 0, m.Rows*m.Cols)
	anyLinkRightUp_AllNode = make([][]int, 0, m.Rows*m.Cols)
	anyLinkRightUp_EdgeStartXIndex = make([][]int, 0, m.Rows*m.Cols)
	//
	println("正在预计算2点经过的坐标点...")
	//从左上到右下点依次遍历 特殊的:原点相同的时候 也会返回空切片
	var slice1, slice2 []int
	for x := 0; x < m.Rows; x++ {
		for y := 0; y < m.Cols; y++ {
			//1.预存储经过的点集合 1级索引是节点索引id 2级索引是经过的xy点整数坐标
			slice1, slice2 = m.judgeLineObstacle_ForPreLoad(0, 0, x, y)
			zeroLinkAny_AllNode = append(zeroLinkAny_AllNode, slice1)
			zeroLinkAny_EdgeStartXIndex = append(zeroLinkAny_EdgeStartXIndex, slice2)
			//
			slice1, slice2 = m.judgeLineObstacle_ForPreLoad(x, y, 0, 0)
			anyLinkZero_AllNode = append(anyLinkZero_AllNode, slice1)
			anyLinkZero_EdgeStartXIndex = append(anyLinkZero_EdgeStartXIndex, slice2)
			//
			slice1, slice2 = m.judgeLineObstacle_ForPreLoad(0, m.Cols-1, x, y)
			rightUpLinkAny_AllNode = append(rightUpLinkAny_AllNode, slice1)
			rightUpLinkAny_EdgeStartXIndex = append(rightUpLinkAny_EdgeStartXIndex, slice2)
			//
			slice1, slice2 = m.judgeLineObstacle_ForPreLoad(x, y, 0, m.Cols-1)
			anyLinkRightUp_AllNode = append(anyLinkRightUp_AllNode, slice1)
			anyLinkRightUp_EdgeStartXIndex = append(anyLinkRightUp_EdgeStartXIndex, slice2)
		}
	}
	//验证 各自互为反向相等
	println("正在验证预存连线一致性...")
	//1.长度验证
	if len(zeroLinkAny_AllNode) != len(anyLinkZero_AllNode) || len(rightUpLinkAny_AllNode) != len(anyLinkRightUp_AllNode) ||
		len(zeroLinkAny_EdgeStartXIndex) != len(anyLinkZero_EdgeStartXIndex) || len(rightUpLinkAny_EdgeStartXIndex) != len(anyLinkRightUp_EdgeStartXIndex) {
		fmt.Println("验证1:当前长度 zeroLinkAny_AllNode:", len(zeroLinkAny_AllNode), " anyLinkZero_AllNode:", len(anyLinkZero_AllNode),
			" rightUpLinkAny_AllNode:", len(rightUpLinkAny_AllNode), " anyLinkRightUp_AllNode:", len(anyLinkRightUp_AllNode))
		panic("连线外围一致性长度错误1")
	}
	//2.连线坐标个数验证
	for i := 0; i < len(zeroLinkAny_AllNode); i++ {
		if len(zeroLinkAny_AllNode[i]) != len(anyLinkZero_AllNode[i]) || len(zeroLinkAny_EdgeStartXIndex[i]) != len(anyLinkZero_EdgeStartXIndex[i]) {
			fmt.Println("验证2,当前id：", i, " 当前长度 zeroLinkAny_AllNode:", len(zeroLinkAny_AllNode[i]), " anyLinkZero_AllNode:", len(anyLinkZero_AllNode[i]),
				" zeroLinkAny_EdgeStartXIndex:", len(zeroLinkAny_EdgeStartXIndex[i]), " anyLinkZero_EdgeStartXIndex:", len(anyLinkZero_EdgeStartXIndex[i]))
			panic("连线内容一致性长度错误2")
		}
	}

	for i := 0; i < len(rightUpLinkAny_AllNode); i++ {
		if len(rightUpLinkAny_AllNode[i]) != len(anyLinkRightUp_AllNode[i]) || len(rightUpLinkAny_EdgeStartXIndex[i]) != len(anyLinkRightUp_EdgeStartXIndex[i]) {
			panic("连线内容一致性长度错误3")
		}
	}
	//
	println("连线一致性通过,当前预计算内存占用(bytes) zeroLinkAny_AllNode:", size.Of(zeroLinkAny_AllNode))
	println("连线一致性通过,当前预计算内存占用(bytes) zeroLinkAny_EdgeStartXIndex:", size.Of(zeroLinkAny_EdgeStartXIndex))
	println("连线一致性通过,当前预计算内存占用(bytes) anyLinkZero_AllNode:", size.Of(anyLinkZero_AllNode))
	println("连线一致性通过,当前预计算内存占用(bytes) anyLinkZero_EdgeStartXIndex:", size.Of(anyLinkZero_EdgeStartXIndex))
	println("连线一致性通过,当前预计算内存占用(bytes) rightUpLinkAny_AllNode:", size.Of(rightUpLinkAny_AllNode))
	println("连线一致性通过,当前预计算内存占用(bytes) rightUpLinkAny_EdgeStartXIndex:", size.Of(rightUpLinkAny_EdgeStartXIndex))
	println("连线一致性通过,当前预计算内存占用(bytes) anyLinkRightUp_AllNode:", size.Of(anyLinkRightUp_AllNode))
	println("连线一致性通过,当前预计算内存占用(bytes) anyLinkRightUp_EdgeStartXIndex:", size.Of(anyLinkRightUp_EdgeStartXIndex))
}

// JudgeLineObstacleNew 判断两点之间连线是否碰到障碍物 默认参数x1, y1, x2, y2都在地图之内 原则:擦边点永远不可能是起点或终点
func (m *Map) judgeLineObstacle_ForPreLoad(x1, y1, x2, y2 int) ([]int, []int) {
	//忘记重置了
	m.clear_BitMap(&m.checkLineRepeat_Slice, m.checkLineRepeat_BitMap)
	//忘记起点终点不能添加进连线经过点
	m.SetSame_BitMap(m.Nodes[x1][y1].singleIndex, &m.checkLineRepeat_Slice, m.checkLineRepeat_BitMap)
	m.SetSame_BitMap(m.Nodes[x2][y2].singleIndex, &m.checkLineRepeat_Slice, m.checkLineRepeat_BitMap)
	//
	var estimateSize int = int(max(math.Abs(float64(x1-x2)), math.Abs(float64(y1-y2))))
	var allNodeSlice []int = make([]int, 0, estimateSize)
	var nowNode *Node
	//judgeNodeSlice := make([]*Node, 0, 20)
	//1.直线同1行
	if x1 == x2 {
		deltaY, newY := 0, 0
		if y1 < y2 {
			//1 -> 2
			deltaY = 1
		} else {
			//2 <- 1
			deltaY = -1
		}
		//
		newY = y1 + deltaY
		for newY > min(y1, y2) && newY < max(y1, y2) {
			//如果直线障碍点有1个是墙 直接返回false
			nowNode = m.Nodes[x1][newY]
			allNodeSlice = append(allNodeSlice, nowNode.X-x1, nowNode.Y-y1)
			newY += deltaY
		}
		//水平垂直没有擦边索引
		return allNodeSlice, make([]int, 0)
	}
	//2.直线同1列
	if y1 == y2 {
		deltaX, newX := 0, 0
		if x1 < x2 {
			//1 start
			//2 big
			deltaX = 1
		} else {
			//2 Small
			//1 start
			deltaX = -1
		}
		newX = x1 + deltaX
		for newX > min(x1, x2) && newX < max(x1, x2) {
			//如果直线障碍点有1个是墙 直接返回false
			nowNode = m.Nodes[newX][y1]
			allNodeSlice = append(allNodeSlice, nowNode.X-x1, nowNode.Y-y1)
			newX += deltaX
		}
		//水平垂直没有擦边索引
		return allNodeSlice, make([]int, 0)
	}
	//不是水平垂直相连才有擦边点
	var edgeNopeSlice []int = make([]int, 0, estimateSize/64)
	//0.先计算dx dy,用后面的点减去前面的点
	dx, dy := x2-x1, y2-y1
	//3.处于标准的正方形对角线2点 判断是不是标准正方形 斜向45度 只选择必经点 只选择必经点 只选择必经点 只选择必经点!!!
	if dx == dy || dx == -dy {
		var nowNode1, nowNode2 *Node
		deltaX, deltaY, newX, newY := 0, 0, x1, y1
		//
		for {
			//2.找擦边点 改变当前点索引
			if dx > 0 && dy > 0 {
				//1.打印右下斜
				//xy1 start
				//    \
				//      xy2终点
				deltaX = 1
				deltaY = 1
			} else if dx > 0 && dy < 0 {
				//2.打印左下斜
				//      xy1 start
				//    /
				//xy2终点
				deltaX = 1
				deltaY = -1
			} else if dx < 0 && dy < 0 {
				//3.打印左上斜
				//xy2
				//    \
				//      xy1 start
				deltaX = -1
				deltaY = -1
			} else if dx < 0 && dy > 0 {
				//4.打印右上斜
				//      xy2
				//    /
				//xy1
				deltaX = -1
				deltaY = 1
			}
			//擦边点起始索引记录 擦边点出现必定记录 而且不需要判断重复
			edgeNopeSlice = append(edgeNopeSlice, len(allNodeSlice))
			//0.先判断擦边点 擦边点肯定在矩形地图范围内
			nowNode1 = m.Nodes[newX+deltaX][newY]
			nowNode2 = m.Nodes[newX][newY+deltaY]
			//1.右和下 擦边点2个都是墙 才不能相连
			allNodeSlice = append(allNodeSlice, nowNode1.X-x1, nowNode1.Y-y1, nowNode2.X-x1, nowNode2.Y-y1)
			//2.再判断下一个必经点是否是终点
			newX += deltaX
			newY += deltaY
			if newX == x2 && newY == y2 {
				return allNodeSlice, edgeNopeSlice //到终点了 代表可以相连
			}
			//3.必经点不重复 标准正方形不需要记忆 擦边点或必经点肯定不重复
			nowNode = m.Nodes[newX][newY]
			allNodeSlice = append(allNodeSlice, nowNode.X-x1, nowNode.Y-y1)
		}
	}
	//0.前面已经计算
	//dx := x2 - x1
	//dy := y2 - y1
	//4.y = kx +b 的情况
	//fmt.Println("处于y = kx +b的情况")
	//1.计算 k 和 b
	k := float64(dy) / float64(dx)
	b := float64(y1) + 0.5 - (k * (float64(x1) + 0.5))
	isYKXBRightUp, isYKXBRightDown, isYKXBLeftDown, isYKXBLeftUp, smallX, smallY, bigX, bigY := false, false, false, false, 0, 0, 0, 0
	var mustLinkNode1, mustLinkNode2, edgeNode1, edgeNode2 *Node
	//判断方向
	//1.打印右下斜
	//xy1 start
	//    \
	//      xy2终点
	//函数图右上
	isYKXBRightUp = dx > 0 && dy > 0
	//2.打印左下斜
	//      xy1 start
	//    /
	//xy2终点
	//函数图右下
	isYKXBRightDown = dx > 0 && dy < 0
	//3.打印左上斜
	//xy2
	//    \
	//      xy1 start
	//函数图左下
	isYKXBLeftDown = dx < 0 && dy < 0
	//4.打印右上斜
	//      xy2
	//    /
	//xy1
	//函数图左上
	isYKXBLeftUp = dx < 0 && dy > 0
	//判断选择哪个轴
	if absInt(dx) > absInt(dy) {
		//选择竖轴作为变量 动态Y判断左右
		focusIntYUp, focusButUp, focusButDown, dynamicY, intDynamicY, deltaX, xStart := 0, false, false, 0.0, 0, 0, 0
		//分类讨论斜率
		if isYKXBRightUp {
			//函数图右上
			//      xy2
			//    /
			//xy1 start
			smallX = x1
			bigX = x2
			xStart, deltaX = smallX+1, 1
		} else if isYKXBRightDown {
			//函数图右下
			//xy1 start
			//    \
			//      xy2
			smallX = x1
			bigX = x2
			xStart, deltaX = smallX+1, 1
		} else if isYKXBLeftDown {
			//函数图左下
			//     xy1 start
			//    /
			//xy2
			smallX = x2
			bigX = x1
			xStart, deltaX = bigX, -1
		} else if isYKXBLeftUp {
			//函数图左上
			//xy2
			//    \
			//    xy1 start
			smallX = x2
			bigX = x1
			xStart, deltaX = bigX, -1
		}
		//通用遍历左右
		for {
			//0.重置当前Y与交点上下判断
			focusButUp = false
			focusButDown = false
			//1.根据不断增加的X获得Y
			dynamicY = (k * float64(xStart)) + b
			intDynamicY = int(dynamicY)
			//2.计算当前Y与当前Y的floor差值
			//3.特殊情况:刚好处于交点 也就是Floor和Ceil相等
			if math.Floor(dynamicY) == math.Ceil(dynamicY) {
				focusButUp, focusButDown = true, true
				focusIntYUp = intDynamicY //当前上Y索引 去掉小数
			} else if dynamicY-math.Floor(dynamicY) < 0.0001 { //focusDis必定大于0
				focusButUp = true         //当前Y值在交点上方
				focusIntYUp = intDynamicY //当前上Y索引
			} else if math.Ceil(dynamicY)-dynamicY <= 0.0001 { //focusDis必定大于0
				focusButDown = true           //当前Y值在交点下方
				focusIntYUp = intDynamicY + 1 //当前上Y索引
			}
			//3.如果差值小于某个数 认为碰到交点
			if focusButUp || focusButDown {
				//分类讨论斜率
				if isYKXBRightUp {
					//函数图右上
					//      xy2
					//    /
					//xy1 start
					//必经点  连通性已确认不是墙
					mustLinkNode1 = m.Nodes[xStart-1][focusIntYUp-1] //中点左下
					mustLinkNode2 = m.Nodes[xStart][focusIntYUp]     //中点右上
					//擦边点 连通性未确认不是墙 //中点左上 //中点右下
					edgeNode1 = m.Nodes[xStart-1][focusIntYUp]
					edgeNode2 = m.Nodes[xStart][focusIntYUp-1]
				} else if isYKXBRightDown {
					//函数图右下
					//xy1 start
					//    \
					//      xy2
					//必经点  连通性已确认不是墙
					mustLinkNode1 = m.Nodes[xStart-1][focusIntYUp] //中点左上
					mustLinkNode2 = m.Nodes[xStart][focusIntYUp-1] //中点右下
					//擦边点 连通性未确认不是墙 					//中点右上 //中点左下
					edgeNode1 = m.Nodes[xStart][focusIntYUp]
					edgeNode2 = m.Nodes[xStart-1][focusIntYUp-1]
				} else if isYKXBLeftDown {
					//函数图左下
					//     xy1 start
					//    /
					//xy2
					//必经点 连通性已确认不是墙
					mustLinkNode1 = m.Nodes[xStart][focusIntYUp]     //中点右上
					mustLinkNode2 = m.Nodes[xStart-1][focusIntYUp-1] //中点左下
					//擦边点 连通性未确认不是墙 					//中点左上 //中点右下
					edgeNode1 = m.Nodes[xStart-1][focusIntYUp]
					edgeNode2 = m.Nodes[xStart][focusIntYUp-1]
				} else if isYKXBLeftUp {
					//函数图左上
					//xy2
					//    \
					//    xy1 start
					//必经点 连通性已确认不是墙
					mustLinkNode1 = m.Nodes[xStart][focusIntYUp-1] //中点右下
					mustLinkNode2 = m.Nodes[xStart-1][focusIntYUp] //中点左上
					//擦边点 连通性未确认不是墙 					//中点左下  //中点右上
					edgeNode1 = m.Nodes[xStart-1][focusIntYUp-1]
					edgeNode2 = m.Nodes[xStart][focusIntYUp]
				}
				//0.先添加必经点1 需要判断重复
				if !m.checkSame_BitMap(mustLinkNode1.singleIndex, m.checkLineRepeat_BitMap) { //必经点1不重复 记录并添加
					allNodeSlice = append(allNodeSlice, mustLinkNode1.X-x1, mustLinkNode1.Y-y1)
					m.SetSame_BitMap(mustLinkNode1.singleIndex, &m.checkLineRepeat_Slice, m.checkLineRepeat_BitMap)
				}
				//1.再添加2个擦边点 不需要判断重复
				edgeNopeSlice = append(edgeNopeSlice, len(allNodeSlice))
				allNodeSlice = append(allNodeSlice, edgeNode1.X-x1, edgeNode1.Y-y1, edgeNode2.X-x1, edgeNode2.Y-y1)
				//2.最后添加必经点2 需要判断重复
				if !m.checkSame_BitMap(mustLinkNode2.singleIndex, m.checkLineRepeat_BitMap) { //必经点2不重复 记录并添加
					allNodeSlice = append(allNodeSlice, mustLinkNode2.X-x1, mustLinkNode2.Y-y1)
					m.SetSame_BitMap(mustLinkNode2.singleIndex, &m.checkLineRepeat_Slice, m.checkLineRepeat_BitMap)
				}
			} else { //6.不处于交点 判断必经点
				//分类讨论斜率
				if isYKXBRightUp {
					//函数图右上
					//      xy2
					//    /
					//xy1 start
					//必经点 连通性已确认不是墙
					mustLinkNode1 = m.Nodes[xStart-1][intDynamicY] //左
					mustLinkNode2 = m.Nodes[xStart][intDynamicY]   //右
				} else if isYKXBRightDown {
					//函数图右下
					//xy1 start
					//    \
					//      xy2
					//必经点 连通性已确认不是墙
					mustLinkNode1 = m.Nodes[xStart-1][intDynamicY] //左
					mustLinkNode2 = m.Nodes[xStart][intDynamicY]   //右
				} else if isYKXBLeftDown {
					//函数图左下
					//     xy1 start
					//    /
					//xy2
					//必经点 连通性已确认不是墙
					mustLinkNode1 = m.Nodes[xStart][intDynamicY]   //右
					mustLinkNode2 = m.Nodes[xStart-1][intDynamicY] //左
				} else if isYKXBLeftUp {
					//函数图左上
					//xy2
					//    \
					//    xy1 start
					//必经点 连通性已确认不是墙
					mustLinkNode1 = m.Nodes[xStart][intDynamicY]   //右
					mustLinkNode2 = m.Nodes[xStart-1][intDynamicY] //左
				}
				//0.先添加必经点1 需要判断重复
				if !m.checkSame_BitMap(mustLinkNode1.singleIndex, m.checkLineRepeat_BitMap) { //必经点1不重复 记录并添加
					allNodeSlice = append(allNodeSlice, mustLinkNode1.X-x1, mustLinkNode1.Y-y1)
					m.SetSame_BitMap(mustLinkNode1.singleIndex, &m.checkLineRepeat_Slice, m.checkLineRepeat_BitMap)
				}
				//2.最后添加必经点2 需要判断重复
				if !m.checkSame_BitMap(mustLinkNode2.singleIndex, m.checkLineRepeat_BitMap) { //必经点2不重复 记录并添加
					allNodeSlice = append(allNodeSlice, mustLinkNode2.X-x1, mustLinkNode2.Y-y1)
					m.SetSame_BitMap(mustLinkNode2.singleIndex, &m.checkLineRepeat_Slice, m.checkLineRepeat_BitMap)
				}
			}
			//4.判断并更新xStart
			if deltaX > 0 { //X向右增加
				if xStart == bigX { //增加到等于最大X(上面已经判断了) 就结束
					break
				}
			} else { //X向左减少
				if xStart == smallX+1 { //减少到等于最小X+1(上面已经判断了) 就结束
					break
				}
			}
			xStart += deltaX
		}
	} else { //dx < dy
		//选择横轴作为变量 判断上下
		focusIntXLeft, focusButLeft, focusButRight, dynamicX, intDynamicX, yStart, deltaY := 0, false, false, 0.0, 0, 0, 0
		//分类讨论斜率
		if isYKXBRightUp {
			//函数图右上
			//      xy2
			//    /
			//xy1 start
			smallY = y1
			bigY = y2
			yStart, deltaY = smallY+1, 1
		} else if isYKXBRightDown {
			//函数图右下
			//xy1 start
			//    \
			//      xy2
			smallY = y2
			bigY = y1
			yStart, deltaY = bigY, -1
		} else if isYKXBLeftDown {
			//函数图左下
			//     xy1 start
			//    /
			//xy2终点
			smallY = y2
			bigY = y1
			yStart, deltaY = bigY, -1
		} else if isYKXBLeftUp {
			//函数图左上
			//xy2
			//    \
			//    xy1 start
			smallY = y1
			bigY = y2
			yStart, deltaY = smallY+1, 1
		}
		//新的 -------------------------------通用遍历上下？？？？？？？？？？？？？？？没写好 这里外循环的Y的变化有增有减
		for {
			//0.重置当前Y与交点上下判断
			focusButLeft = false
			focusButRight = false
			//1.根据不断增加的Y获得X
			dynamicX = (float64(yStart) - b) / k
			intDynamicX = int(dynamicX)
			//2.计算当前X与当前X的floor差值
			if math.Ceil(dynamicX) == math.Floor(dynamicX) {
				focusButLeft, focusButRight = true, true
				focusIntXLeft = intDynamicX - 1 //当前左X索引
			} else if math.Ceil(dynamicX)-dynamicX < 0.0001 { //focusDis必定大于0
				focusButLeft = true         //当前X值在交点左
				focusIntXLeft = intDynamicX //当前左X索引
			} else if dynamicX-math.Floor(dynamicX) < 0.0001 { //focusDis必定大于0
				focusButRight = true            //当前X值在交点右
				focusIntXLeft = intDynamicX - 1 //当前左X索引
			}
			//3.如果差值小于某个数或相等 认为碰到交点 //不要重复点 midNotWallNodeSliceFromX1按顺序添加
			if focusButLeft || focusButRight {
				//分类讨论斜率
				if isYKXBRightUp {
					//函数图右上
					//      xy2
					//    /
					//xy1 start
					//必经点  连通性已确认不是墙
					mustLinkNode1 = m.Nodes[focusIntXLeft][yStart-1] //中点左下
					mustLinkNode2 = m.Nodes[focusIntXLeft+1][yStart] //中点右上
					//擦边点 连通性未确认不是墙
					edgeNode1 = m.Nodes[focusIntXLeft+1][yStart-1]
					edgeNode2 = m.Nodes[focusIntXLeft][yStart]
				} else if isYKXBRightDown {
					//函数图右下
					//xy1 start
					//    \
					//      xy2
					//必经点
					mustLinkNode1 = m.Nodes[focusIntXLeft][yStart]     //中点左上
					mustLinkNode2 = m.Nodes[focusIntXLeft+1][yStart-1] //中点右下
					//擦边点 连通性未确认不是墙
					edgeNode1 = m.Nodes[focusIntXLeft+1][yStart]
					edgeNode2 = m.Nodes[focusIntXLeft][yStart-1]
				} else if isYKXBLeftDown {
					//函数图左下
					//     xy1 start
					//    /
					//xy2终点
					//必经点
					mustLinkNode1 = m.Nodes[focusIntXLeft+1][yStart] //中点右上
					mustLinkNode2 = m.Nodes[focusIntXLeft][yStart-1] //中点左下
					//擦边点 连通性未确认不是墙
					edgeNode1 = m.Nodes[focusIntXLeft][yStart]
					edgeNode2 = m.Nodes[focusIntXLeft+1][yStart-1]
				} else if isYKXBLeftUp {
					//函数图左上
					//xy2
					//    \
					//    xy1 start
					//必经点
					mustLinkNode1 = m.Nodes[focusIntXLeft+1][yStart-1] //中点右下
					mustLinkNode2 = m.Nodes[focusIntXLeft][yStart]     //中点左上
					//擦边点 连通性未确认不是墙 //中点右上 //中点左下
					edgeNode1 = m.Nodes[focusIntXLeft+1][yStart]
					edgeNode2 = m.Nodes[focusIntXLeft][yStart-1]
				}
				//0.先添加必经点1 需要判断重复
				if !m.checkSame_BitMap(mustLinkNode1.singleIndex, m.checkLineRepeat_BitMap) { //必经点1不重复 记录并添加
					allNodeSlice = append(allNodeSlice, mustLinkNode1.X-x1, mustLinkNode1.Y-y1)
					m.SetSame_BitMap(mustLinkNode1.singleIndex, &m.checkLineRepeat_Slice, m.checkLineRepeat_BitMap)
				}
				//1.再添加2个擦边点 不需要判断重复
				edgeNopeSlice = append(edgeNopeSlice, len(allNodeSlice))
				allNodeSlice = append(allNodeSlice, edgeNode1.X-x1, edgeNode1.Y-y1, edgeNode2.X-x1, edgeNode2.Y-y1)
				//2.最后添加必经点2 需要判断重复
				if !m.checkSame_BitMap(mustLinkNode2.singleIndex, m.checkLineRepeat_BitMap) { //必经点2不重复 记录并添加
					allNodeSlice = append(allNodeSlice, mustLinkNode2.X-x1, mustLinkNode2.Y-y1)
					m.SetSame_BitMap(mustLinkNode2.singleIndex, &m.checkLineRepeat_Slice, m.checkLineRepeat_BitMap)
				}
			} else { //6.不处于交点 判断int点就行
				//分类讨论斜率
				if isYKXBRightUp {
					//函数图右上
					//      xy2
					//    /
					//xy1 start
					//必经点 连通性已确认不是墙
					mustLinkNode1 = m.Nodes[intDynamicX][yStart-1] //下
					mustLinkNode2 = m.Nodes[intDynamicX][yStart]   //上
				} else if isYKXBRightDown {
					//函数图右下
					//xy1 start
					//    \
					//      xy2
					//必经点 连通性已确认不是墙
					mustLinkNode1 = m.Nodes[intDynamicX][yStart]   //上
					mustLinkNode2 = m.Nodes[intDynamicX][yStart-1] //下
				} else if isYKXBLeftDown {
					//函数图左下
					//     xy1 start
					//    /
					//xy2终点
					//必经点 连通性已确认不是墙
					mustLinkNode1 = m.Nodes[intDynamicX][yStart]   //上
					mustLinkNode2 = m.Nodes[intDynamicX][yStart-1] //下
				} else if isYKXBLeftUp {
					//函数图左上
					//xy2
					//    \
					//    xy1 start
					//必经点 连通性已确认不是墙
					mustLinkNode1 = m.Nodes[intDynamicX][yStart-1] //下
					mustLinkNode2 = m.Nodes[intDynamicX][yStart]   //上
				}
				//0.先添加必经点1 需要判断重复
				if !m.checkSame_BitMap(mustLinkNode1.singleIndex, m.checkLineRepeat_BitMap) { //必经点1不重复 记录并添加
					allNodeSlice = append(allNodeSlice, mustLinkNode1.X-x1, mustLinkNode1.Y-y1)
					m.SetSame_BitMap(mustLinkNode1.singleIndex, &m.checkLineRepeat_Slice, m.checkLineRepeat_BitMap)
				}
				//2.最后添加必经点2 需要判断重复
				if !m.checkSame_BitMap(mustLinkNode2.singleIndex, m.checkLineRepeat_BitMap) { //必经点2不重复 记录并添加
					allNodeSlice = append(allNodeSlice, mustLinkNode2.X-x1, mustLinkNode2.Y-y1)
					m.SetSame_BitMap(mustLinkNode2.singleIndex, &m.checkLineRepeat_Slice, m.checkLineRepeat_BitMap)
				}
			}
			//4.判断并更新xStart
			if deltaY > 0 { //y向上增加
				if yStart == bigY { //增加到等于最大Y(上面已经判断了) 就结束
					break
				}
			} else { //Y向下减少
				if yStart == smallY+1 { //减少到等于最小Y+1(上面已经判断了) 就结束
					break
				}
			}
			yStart += deltaY
		}
	}
	//最终y=kx+b没有false则判定相连
	return allNodeSlice, edgeNopeSlice
}

func getAbsInt(a int) int {
	if a >= 0 {
		return a
	}
	return -a
}
func (m *Map) JudgeLineObstacleNew_GetOneEdgeObstacle_UsePreload(x1, y1, x2, y2 int) (bool, *Node) {
	//0.同一点阻断
	if x1 == x2 && y1 == y2 {
		panic("传入同一点检测 请排查")
	}
	//zeroLinkAny_AllNode
	//zeroLinkAny_EdgeStartXIndex
	//rightUpLinkAny_AllNode
	//rightUpLinkAny_EdgeStartXIndex
	//1.以x1点为中心点 判断x2在x1点的哪个方向 有8个方向 -水平垂直优先判断
	var allNodeSlice, EdgeStartXIndexSlice [][]int
	dx := x2 - x1
	dy := y2 - y1
	absDx := getAbsInt(dx)
	abdDy := getAbsInt(dy)
	singleId := 0
	//水平或垂直相连情况：
	if absDx == 0 { //同一行
		//1.判断x2在右还是左
		if dy > 0 { //xy1 -> xy2 singleIndex为原点
			singleId = m.Nodes[absDx][abdDy].singleIndex
			allNodeSlice, EdgeStartXIndexSlice = zeroLinkAny_AllNode, zeroLinkAny_EdgeStartXIndex
		} else { //xy2 <- xy1  singleIndex为右上点
			singleId = m.Nodes[absDx][m.Cols-1-abdDy].singleIndex
			allNodeSlice, EdgeStartXIndexSlice = rightUpLinkAny_AllNode, rightUpLinkAny_EdgeStartXIndex
		}
		//fmt.Println("--当前x1:", x1, " 当前y1:", y1, " 当前x2:", x2, " 当前y2:", y2)
		return m.normal_JudgeLine(allNodeSlice, EdgeStartXIndexSlice, singleId, x1, y1)
	}
	if abdDy == 0 { //同一列
		if dx > 0 {
			//xy1 //偏移x1为准
			//↓
			//xy2
			singleId = m.Nodes[absDx][abdDy].singleIndex
			allNodeSlice, EdgeStartXIndexSlice = zeroLinkAny_AllNode, zeroLinkAny_EdgeStartXIndex
		} else {
			//xy2 //偏移x2为准 但是反向
			//↑
			//xy1
			singleId = m.Nodes[absDx][abdDy].singleIndex
			allNodeSlice, EdgeStartXIndexSlice = anyLinkZero_AllNode, anyLinkZero_EdgeStartXIndex
		}
		//fmt.Println("--当前x1:", x1, " 当前y1:", y1, " 当前x2:", x2, " 当前y2:", y2)
		return m.normal_JudgeLine(allNodeSlice, EdgeStartXIndexSlice, singleId, x1, y1)
	}
	//不是水平或垂直情况 先判断x2到底在哪个斜向 -- 就是起始点的xy+实际的xy差 = 偏移点xy
	//原点在左上角
	if dx > 0 && dy > 0 { //zeroLinkAny_AllNode 正向 偏移为xy1
		//xy1 start
		//    \
		//      xy2终点
		allNodeSlice = zeroLinkAny_AllNode
		EdgeStartXIndexSlice = zeroLinkAny_EdgeStartXIndex
		singleId = m.Nodes[absDx][abdDy].singleIndex
	} else if dx > 0 && dy < 0 { //rightUpLinkAny_AllNode 正向
		//      xy1 start
		//    /
		//xy2终点
		allNodeSlice = rightUpLinkAny_AllNode
		EdgeStartXIndexSlice = rightUpLinkAny_EdgeStartXIndex
		singleId = m.Nodes[absDx][m.Cols-1-abdDy].singleIndex
	} else if dx < 0 && dy < 0 {
		//xy2
		//    \
		//      xy1 start
		allNodeSlice = anyLinkZero_AllNode
		EdgeStartXIndexSlice = anyLinkZero_EdgeStartXIndex
		singleId = m.Nodes[absDx][abdDy].singleIndex
	} else if dx < 0 && dy > 0 {
		//      xy2
		//    /
		//xy1
		allNodeSlice = anyLinkRightUp_AllNode
		EdgeStartXIndexSlice = anyLinkRightUp_EdgeStartXIndex
		singleId = m.Nodes[absDx][m.Cols-1-abdDy].singleIndex
	}
	//fmt.Println("--当前x1:", x1, " 当前y1:", y1, " 当前x2:", x2, " 当前y2:", y2)
	return m.normal_JudgeLine(allNodeSlice, EdgeStartXIndexSlice, singleId, x1, y1)
}

func (m *Map) normal_JudgeLine(allNodeSlice, edgeNodeSlice [][]int, singleId, x1, y1 int) (bool, *Node) {
	var edgeNode1, edgeNode2, mustNode *Node
	var allIndex, nowEdgeIndex, jx1, jy1, jx2, jy2 = 0, 0, 0, 0, 0, 0
	//1.先判断当前是否有擦边点
	if len(edgeNodeSlice[singleId]) > 0 { //当前连线有擦边点
		for { //-1是因为存的是整数坐标 倒数第2个是x坐标 倒数第1个是y坐标 --注意i范围是到倒数第2个
			//1.先判断当前索引是否是擦边点起始索引 越界:当前正向 边界索引小于长度
			if nowEdgeIndex < len(edgeNodeSlice[singleId]) && edgeNodeSlice[singleId][nowEdgeIndex] == allIndex { //当前是擦边点起始索引 擦边点是斜边2点
				jx1 = allNodeSlice[singleId][allIndex] + x1
				jy1 = allNodeSlice[singleId][allIndex+1] + y1
				jx2 = allNodeSlice[singleId][allIndex+2] + x1
				jy2 = allNodeSlice[singleId][allIndex+3] + y1
				edgeNode1 = m.Nodes[jx1][jy1]
				edgeNode2 = m.Nodes[jx2][jy2]
				if edgeNode1.IsWall() && edgeNode2.IsWall() { //2个擦边点都是墙才构成有效阻挡
					return false, edgeNode1
				}
				//end:边界索引右移
				nowEdgeIndex++
				allIndex += 4 //下1个起始判断索引
			} else { //当前索引不是擦边点 那么是必经点
				//fmt.Println("当前singleId：", singleId, " 当前allIndex：", allIndex)
				jx1 = allNodeSlice[singleId][allIndex] + x1
				jy1 = allNodeSlice[singleId][allIndex+1] + y1
				mustNode = m.Nodes[jx1][jy1]
				if mustNode.IsWall() {
					return false, mustNode //有障碍代表不可以相连 返回碰到的第1个障碍
				}
				allIndex += 2 //下1个起始判断索引
			}
			//end:判断索引是否到边界
			if allIndex >= len(allNodeSlice[singleId])-1 { //allIndex不能是倒数第1个索引
				return true, nil
			}
		}
	} else { //当前连线无擦边点 全是必经点
		for i := 0; i < len(allNodeSlice[singleId])-1; i += 2 { //-1是因为存的是整数坐标 倒数第2个是x坐标 倒数第1个是y坐标 --注意i范围是到倒数第2个
			jx1 = allNodeSlice[singleId][i] + x1
			jy1 = allNodeSlice[singleId][i+1] + y1
			//fmt.Println("当前singleId：", singleId, " 当前i：", i)
			//fmt.Println("当前jx1：", jx1, " 当前jy1：", jy1)
			//if jy1 == -1 {
			//	fmt.Println("进来")
			//}
			mustNode = m.Nodes[jx1][jy1]
			if mustNode.IsWall() {
				return false, mustNode //有障碍代表不可以相连 返回碰到的第1个障碍
			}
		}
	}
	//默认返回无障碍
	return true, nil
}

// JudgeLineObstacleNew 判断两点之间连线是否碰到障碍物 默认参数x1, y1, x2, y2都在地图之内 version0.0.1:修改45度的双向判断(原来某个阻断 现在必须2个一起才阻断)
func (m *Map) JudgeLineObstacleNew(x1, y1, x2, y2 int) bool {
	//这一步 传入指针切片，内部把不是墙的节点添加完毕后传给外部
	var nowNode *Node
	//judgeNodeSlice := make([]*Node, 0, 20)
	//1.直线同1行
	if x1 == x2 {
		if y1 < y2 {
			//1start  2
			for newY := y1 + 1; newY < y2; newY++ {
				//如果直线障碍点有1个是墙 直接返回false
				nowNode = m.Nodes[x1][newY]
				if nowNode.IsWall() {
					return false //
				}
			}
		} else {
			//2 1start
			for newY := y1 - 1; newY > y2; newY-- {
				//如果直线障碍点有1个是墙 直接返回false
				nowNode = m.Nodes[x1][newY]
				if nowNode.IsWall() {
					return false //
				}
			}
		}
		//都不是墙 代表可以相连
		return true
	}
	//2.直线同1列
	if y1 == y2 {
		if x1 < x2 {
			//1 Small start
			//2 big
			for newX := x1 + 1; newX < x2; newX++ {
				//如果直线障碍点有1个是墙 直接返回false
				nowNode = m.Nodes[newX][y1]
				if nowNode.IsWall() {
					return false //
				}
			}
		} else {
			//2 Small
			//1 big start
			for newX := x1 - 1; newX > x2; newX-- {
				//如果直线障碍点有1个是墙 直接返回false
				nowNode = m.Nodes[newX][y1]
				if nowNode.IsWall() {
					return false //
				}
			}
		}
		//都不是墙 代表可以相连
		return true
	}
	//0.先计算dx dy,用后面的点减去前面的点
	dx := x2 - x1
	dy := y2 - y1
	//3.处于标准的正方形对角线2点 判断是不是标准正方形 斜向45度 只选择必经点 只选择必经点 只选择必经点 只选择必经点!!!
	if dx == dy || dx == -dy {
		newX := x1
		newY := y1
		var nowNode1 *Node
		var nowNode2 *Node
		//
		var deltaX, deltaY int
		//
		for {
			//2.找擦边点 改变当前点索引
			if dx > 0 && dy > 0 {
				//1.打印右下斜
				//xy1 start
				//    \
				//      xy2终点
				deltaX = 1
				deltaY = 1
			} else if dx > 0 && dy < 0 {
				//2.打印左下斜
				//      xy1 start
				//    /
				//xy2终点
				deltaX = 1
				deltaY = -1
			} else if dx < 0 && dy < 0 {
				//3.打印左上斜
				//xy2
				//    \
				//      xy1 start
				deltaX = -1
				deltaY = -1
			} else if dx < 0 && dy > 0 {
				//4.打印右上斜
				//      xy2
				//    /
				//xy1
				deltaX = -1
				deltaY = 1
			}
			//0.先判断擦边点 擦边点肯定在矩形地图范围内
			nowNode1 = m.Nodes[newX+deltaX][newY]
			nowNode2 = m.Nodes[newX][newY+deltaY]
			//1.右和下 擦边点2个都是墙 才不能相连
			if nowNode1.IsWall() && nowNode2.IsWall() {
				return false
			}
			//2.再判断下一个必经点是否是终点
			newX += deltaX
			newY += deltaY
			if newX == x2 && newY == y2 {
				return true //到终点了 代表可以相连
			}
			//3.必经点
			nowNode = m.Nodes[newX][newY]
			if nowNode.IsWall() {
				return false
			}
		}
	}
	//0.前面已经计算
	//dx := x2 - x1
	//dy := y2 - y1
	//4.y = kx +b 的情况
	//fmt.Println("处于y = kx +b的情况")
	//1.计算 k 和 b
	k, b := 0.0, 0.0
	k = float64(dy) / float64(dx)
	b = float64(y1) + 0.5 - (k * (float64(x1) + 0.5))
	isYKXBRightUp := false
	isYKXBRightDown := false
	isYKXBLeftDown := false
	isYKXBLeftUp := false
	//判断方向
	if dx > 0 && dy > 0 {
		//1.打印右下斜
		//xy1 start
		//    \
		//      xy2终点
		//函数图右上
		isYKXBRightUp = true
	} else if dx > 0 && dy < 0 {
		//2.打印左下斜
		//      xy1 start
		//    /
		//xy2终点
		//函数图右下
		isYKXBRightDown = true
	} else if dx < 0 && dy < 0 {
		//3.打印左上斜
		//xy2
		//    \
		//      xy1 start
		//函数图左下
		isYKXBLeftDown = true
	} else if dx < 0 && dy > 0 {
		//4.打印右上斜
		//      xy2
		//    /
		//xy1
		//函数图左上
		isYKXBLeftUp = true
	}
	//
	smallX, smallY, bigX, bigY := 0, 0, 0, 0
	//判断选择哪个轴
	if absInt(dx) > absInt(dy) {
		//选择x轴作为变量 动态Y判断左右
		focusIntYUp := 0
		focusButUp := false
		focusButDown := false
		dynamicY := 0.0
		intDynamicY := 0
		//判断函数图方向中,谁的索引差大就选择索引轴(X或Y轴)作为遍历去计算另一个值
		if isYKXBRightUp {
			//函数图右上
			//      xy2
			//    /
			//xy1 start
			smallX = x1
			bigX = x2
			//通用遍历左右
			for XRight := smallX + 1; XRight <= bigX; XRight++ {
				//0.重置当前Y与交点上下判断
				focusButUp = false
				focusButDown = false
				//1.根据不断增加的X获得Y
				dynamicY = (k * float64(XRight)) + b
				intDynamicY = int(dynamicY)
				//2.计算当前Y与当前Y的floor差值
				//3.特殊情况:刚好处于交点 也就是Floor和Ceil相等
				if math.Floor(dynamicY) == math.Ceil(dynamicY) {
					focusButUp, focusButDown = true, true
					focusIntYUp = intDynamicY //当前上Y索引 去掉小数
				} else if dynamicY-math.Floor(dynamicY) < 0.01 { //focusDis必定大于0
					focusButUp = true         //当前Y值在交点上方
					focusIntYUp = intDynamicY //当前上Y索引
				} else if math.Ceil(dynamicY)-dynamicY <= 0.01 { //focusDis必定大于0
					focusButDown = true           //当前Y值在交点下方
					focusIntYUp = intDynamicY + 1 //当前上Y索引
				}
				//3.如果差值小于某个数 认为碰到交点
				if focusButUp || focusButDown {
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[XRight-1][focusIntYUp-1] //中点左下
					if nowNode.IsWall() {
						return false
					}
					//擦边点 连通性未确认不是墙
					if m.Nodes[XRight-1][focusIntYUp].IsWall() && m.Nodes[XRight][focusIntYUp-1].IsWall() { //中点左上 //中点右下
						return false
					}
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight][focusIntYUp] //中点右上
					if nowNode.IsWall() {
						return false
					}
				} else {
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight-1][intDynamicY] //左
					if nowNode.IsWall() {
						return false
					}
					nowNode = m.Nodes[XRight][intDynamicY] //右
					if nowNode.IsWall() {
						return false
					}
				}
			}
		} else if isYKXBRightDown {
			//函数图右下
			//xy1 start
			//    \
			//      xy2
			smallX = x1
			bigX = x2
			//通用遍历左右
			for XRight := smallX + 1; XRight <= bigX; XRight++ {
				//0.重置当前Y与交点上下判断
				focusButUp = false
				focusButDown = false
				//1.根据不断增加的X获得Y
				dynamicY = (k * float64(XRight)) + b
				intDynamicY = int(dynamicY)
				//2.计算当前Y与当前Y的floor差值
				//3.特殊情况:刚好处于交点 也就是Floor和Ceil相等
				if math.Floor(dynamicY) == math.Ceil(dynamicY) {
					focusButUp, focusButDown = true, true
					focusIntYUp = intDynamicY //当前上Y索引 去掉小数
				} else if dynamicY-math.Floor(dynamicY) < 0.01 { //focusDis必定大于0
					focusButUp = true         //当前Y值在交点上方
					focusIntYUp = intDynamicY //当前上Y索引
				} else if math.Ceil(dynamicY)-dynamicY <= 0.01 { //focusDis必定大于0
					focusButDown = true           //当前Y值在交点下方
					focusIntYUp = intDynamicY + 1 //当前上Y索引
				}
				//3.如果差值小于某个数 认为碰到交点
				if focusButUp || focusButDown {
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[XRight-1][focusIntYUp] //中点左上
					if nowNode.IsWall() {
						return false
					}
					//擦边点 连通性未确认不是墙
					if m.Nodes[XRight][focusIntYUp].IsWall() && m.Nodes[XRight-1][focusIntYUp-1].IsWall() { //中点右上 //中点左下
						return false
					}
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight][focusIntYUp-1] //中点右下
					if nowNode.IsWall() {
						return false
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight-1][intDynamicY] //左
					if nowNode.IsWall() {
						return false
					}
					nowNode = m.Nodes[XRight][intDynamicY] //右
					if nowNode.IsWall() {
						return false
					}
				}
			}
		} else if isYKXBLeftDown {
			//函数图左下
			//     xy1 start
			//    /
			//xy2
			smallX = x2
			bigX = x1
			//通用遍历左右
			for XRight := bigX; XRight > smallX; XRight-- {
				//0.重置当前Y与交点上下判断
				focusButUp = false
				focusButDown = false
				//1.根据不断增加的X获得Y
				dynamicY = (k * float64(XRight)) + b
				intDynamicY = int(dynamicY)
				//2.计算当前Y与当前Y的floor差值
				//3.特殊情况:刚好处于交点 也就是Floor和Ceil相等
				if math.Floor(dynamicY) == math.Ceil(dynamicY) {
					focusButUp, focusButDown = true, true
					focusIntYUp = intDynamicY //当前上Y索引 去掉小数
				} else if dynamicY-math.Floor(dynamicY) < 0.01 { //focusDis必定大于0
					focusButUp = true         //当前Y值在交点上方
					focusIntYUp = intDynamicY //当前上Y索引
				} else if math.Ceil(dynamicY)-dynamicY <= 0.01 { //focusDis必定大于0
					focusButDown = true           //当前Y值在交点下方
					focusIntYUp = intDynamicY + 1 //当前上Y索引
				}
				//3.如果差值小于某个数 认为碰到交点
				if focusButUp || focusButDown {
					//4.判断必经点 擦边点 注意：不同方向必经点不一样
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][focusIntYUp-1], m.Nodes[XRight][focusIntYUp],
					//	m.Nodes[XRight-1][focusIntYUp], m.Nodes[XRight][focusIntYUp-1])
					//5.如果当前方向是 /
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight][focusIntYUp] //中点右上
					if nowNode.IsWall() {
						return false
					}
					//擦边点 连通性未确认不是墙
					if m.Nodes[XRight-1][focusIntYUp].IsWall() && m.Nodes[XRight][focusIntYUp-1].IsWall() { //中点左上 //中点右下
						return false
					}
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[XRight-1][focusIntYUp-1] //中点左下
					if nowNode.IsWall() {
						return false
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight][intDynamicY] //右
					if nowNode.IsWall() {
						return false
					}
					nowNode = m.Nodes[XRight-1][intDynamicY] //左
					if nowNode.IsWall() {
						return false
					}
				}
			}
		} else if isYKXBLeftUp {
			//函数图左上
			//xy2
			//    \
			//    xy1 start
			smallX = x2
			bigX = x1
			//通用遍历左右
			for XRight := bigX; XRight > smallX; XRight-- {
				//0.重置当前Y与交点上下判断
				focusButUp = false
				focusButDown = false
				//1.根据不断增加的X获得Y
				dynamicY = (k * float64(XRight)) + b
				intDynamicY = int(dynamicY)
				//2.计算当前Y与当前Y的floor差值
				//3.特殊情况:刚好处于交点 也就是Floor和Ceil相等
				if math.Floor(dynamicY) == math.Ceil(dynamicY) {
					focusButUp, focusButDown = true, true
					focusIntYUp = intDynamicY //当前上Y索引 去掉小数
				} else if dynamicY-math.Floor(dynamicY) < 0.01 { //focusDis必定大于0
					focusButUp = true         //当前Y值在交点上方
					focusIntYUp = intDynamicY //当前上Y索引
				} else if math.Ceil(dynamicY)-dynamicY <= 0.01 { //focusDis必定大于0
					focusButDown = true           //当前Y值在交点下方
					focusIntYUp = intDynamicY + 1 //当前上Y索引
				}
				//3.如果差值小于某个数 认为碰到交点
				if focusButUp || focusButDown {
					//4.判断必经点 擦边点 注意：不同方向必经点不一样
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][focusIntYUp-1], m.Nodes[XRight][focusIntYUp],
					//	m.Nodes[XRight-1][focusIntYUp], m.Nodes[XRight][focusIntYUp-1])
					//5.如果当前方向是 /
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight][focusIntYUp-1] //中点右下
					if nowNode.IsWall() {
						return false
					}
					//擦边点 连通性未确认不是墙
					if m.Nodes[XRight-1][focusIntYUp-1].IsWall() && m.Nodes[XRight][focusIntYUp].IsWall() { //中点左下  //中点右上
						return false
					}
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[XRight-1][focusIntYUp] //中点左上
					if nowNode.IsWall() {
						return false
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight][intDynamicY] //右
					if nowNode.IsWall() {
						return false
					}
					nowNode = m.Nodes[XRight-1][intDynamicY] //左
					if nowNode.IsWall() {
						return false
					}
				}
			}
		}
	} else { //dx < dy
		//选择y轴作为变量 判断上下
		focusIntXLeft := 0
		focusButLeft := false
		focusButRight := false
		dynamicX := 0.0
		intDynamicX := 0
		//判断函数图方向中,谁的索引差大就选择索引轴(X或Y轴)作为遍历去计算另一个值
		if isYKXBRightUp {
			//函数图右上
			//      xy2
			//    /
			//xy1 start
			smallY = y1
			bigY = y2
			//通用遍历上下
			for YUp := smallY + 1; YUp <= bigY; YUp++ {
				//0.重置当前Y与交点上下判断
				focusButLeft = false
				focusButRight = false
				//1.根据不断增加的Y获得X
				dynamicX = (float64(YUp) - b) / k
				intDynamicX = int(dynamicX)
				//2.计算当前X与当前X的floor差值
				if math.Ceil(dynamicX) == math.Floor(dynamicX) {
					focusButLeft, focusButRight = true, true
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				} else if math.Ceil(dynamicX)-dynamicX < 0.01 { //focusDis必定大于0
					focusButLeft = true         //当前X值在交点左
					focusIntXLeft = intDynamicX //当前左X索引
				} else if dynamicX-math.Floor(dynamicX) < 0.01 { //focusDis必定大于0
					focusButRight = true            //当前X值在交点右
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				}
				//3.如果差值小于某个数或相等 认为碰到交点
				if focusButLeft || focusButRight {
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft][YUp-1] //中点左下
					if nowNode.IsWall() {
						return false
					}
					//擦边点 连通性未确认不是墙
					if m.Nodes[focusIntXLeft+1][YUp-1].IsWall() && m.Nodes[focusIntXLeft][YUp].IsWall() { //中点右下 //中点左上
						return false
					}
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft+1][YUp] //中点右上
					if nowNode.IsWall() {
						return false
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[intDynamicX][YUp-1] //下
					if nowNode.IsWall() {
						return false
					}
					nowNode = m.Nodes[intDynamicX][YUp] //上
					if nowNode.IsWall() {
						return false
					}
				}
			}
		} else if isYKXBRightDown {
			//函数图右下
			//xy1 start
			//    \
			//      xy2
			smallY = y2
			bigY = y1
			//通用遍历上下
			for YUp := bigY; YUp > smallY; YUp-- {
				//0.重置当前Y与交点上下判断
				focusButLeft = false
				focusButRight = false
				//1.根据不断增加的Y获得X
				dynamicX = (float64(YUp) - b) / k
				intDynamicX = int(dynamicX)
				//2.计算当前X与当前X的floor差值
				if math.Ceil(dynamicX) == math.Floor(dynamicX) {
					focusButLeft, focusButRight = true, true
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				} else if math.Ceil(dynamicX)-dynamicX < 0.01 { //focusDis必定大于0
					focusButLeft = true         //当前X值在交点左
					focusIntXLeft = intDynamicX //当前左X索引
				} else if dynamicX-math.Floor(dynamicX) < 0.01 { //focusDis必定大于0
					focusButRight = true            //当前X值在交点右
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				}
				//3.如果差值小于某个数或相等 认为碰到交点
				if focusButLeft || focusButRight {
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft][YUp] //中点左上
					if nowNode.IsWall() {
						return false
					}
					//擦边点 连通性未确认不是墙
					if m.Nodes[focusIntXLeft+1][YUp].IsWall() && m.Nodes[focusIntXLeft][YUp-1].IsWall() { //中点右上  //中点左下
						return false
					}
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft+1][YUp-1] //中点右下
					if nowNode.IsWall() {
						return false
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[intDynamicX][YUp] //上
					if nowNode.IsWall() {
						return false
					}
					nowNode = m.Nodes[intDynamicX][YUp-1] //下
					if nowNode.IsWall() {
						return false
					}
				}
			}
		} else if isYKXBLeftDown {
			//函数图左下
			//     xy1 start
			//    /
			//xy2终点
			smallY = y2
			bigY = y1
			//通用遍历上下
			for YUp := bigY; YUp > smallY; YUp-- {
				//0.重置当前Y与交点上下判断
				focusButLeft = false
				focusButRight = false
				//1.根据不断增加的Y获得X
				dynamicX = (float64(YUp) - b) / k
				intDynamicX = int(dynamicX)
				//2.计算当前X与当前X的floor差值
				if math.Ceil(dynamicX) == math.Floor(dynamicX) {
					focusButLeft, focusButRight = true, true
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				} else if math.Ceil(dynamicX)-dynamicX < 0.01 { //focusDis必定大于0
					focusButLeft = true         //当前X值在交点左
					focusIntXLeft = intDynamicX //当前左X索引
				} else if dynamicX-math.Floor(dynamicX) < 0.01 { //focusDis必定大于0
					focusButRight = true            //当前X值在交点右
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				}
				//3.如果差值小于某个数或相等 认为碰到交点
				if focusButLeft || focusButRight {
					//4.判断必经点 擦边点 注意：不同方向必经点不一样
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][focusIntYUp-1], m.Nodes[XRight][focusIntYUp],
					//	m.Nodes[XRight-1][focusIntYUp], m.Nodes[XRight][focusIntYUp-1])
					//5.如果当前方向是 /
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft+1][YUp] //中点右上
					if nowNode.IsWall() {
						return false
					}
					//擦边点 连通性未确认不是墙
					if m.Nodes[focusIntXLeft][YUp].IsWall() && m.Nodes[focusIntXLeft+1][YUp-1].IsWall() { //中点左上 //中点右下
						return false
					}
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft][YUp-1] //中点左下
					if nowNode.IsWall() {
						return false
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[intDynamicX][YUp] //上
					if nowNode.IsWall() {
						return false
					}
					nowNode = m.Nodes[intDynamicX][YUp-1] //下
					if nowNode.IsWall() {
						return false
					}
				}
			}
		} else if isYKXBLeftUp {
			//函数图左上
			//xy2
			//    \
			//    xy1 start
			smallY = y1
			bigY = y2
			//通用遍历上下
			for YUp := smallY + 1; YUp <= bigY; YUp++ {
				//0.重置当前Y与交点上下判断
				focusButLeft = false
				focusButRight = false
				//1.根据不断增加的Y获得X
				dynamicX = (float64(YUp) - b) / k
				intDynamicX = int(dynamicX)
				//2.计算当前X与当前X的floor差值
				if math.Ceil(dynamicX) == math.Floor(dynamicX) {
					focusButLeft, focusButRight = true, true
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				} else if math.Ceil(dynamicX)-dynamicX < 0.01 { //focusDis必定大于0
					focusButLeft = true         //当前X值在交点左
					focusIntXLeft = intDynamicX //当前左X索引
				} else if dynamicX-math.Floor(dynamicX) < 0.01 { //focusDis必定大于0
					focusButRight = true            //当前X值在交点右
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				}
				//3.如果差值小于某个数或相等 认为碰到交点
				if focusButLeft || focusButRight {
					//4.判断必经点 擦边点 注意：不同方向必经点不一样
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][focusIntYUp-1], m.Nodes[XRight][focusIntYUp],
					//	m.Nodes[XRight-1][focusIntYUp], m.Nodes[XRight][focusIntYUp-1])
					//5.如果当前方向是 /
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft+1][YUp-1] //中点右下
					if nowNode.IsWall() {
						return false
					}
					//擦边点 连通性未确认不是墙
					if m.Nodes[focusIntXLeft+1][YUp].IsWall() && m.Nodes[focusIntXLeft][YUp-1].IsWall() { //中点右上 //中点左下
						return false
					}
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft][YUp] //中点左上
					if nowNode.IsWall() {
						return false
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[intDynamicX][YUp-1] //下
					if nowNode.IsWall() {
						return false
					}
					nowNode = m.Nodes[intDynamicX][YUp] //上
					if nowNode.IsWall() {
						return false
					}
				}
			}
		}
	}
	//最终y=kx+b没有false则判定相连
	return true
}

// JudgeLineObstacleNew_GetOneEdgeObstacle 判断两点之间连线是否碰到障碍物 返回碰到的第1个障碍 默认参数x1, y1, x2, y2都在地图之内 version0.0.1:修改45度的双向判断(原来某个阻断 现在必须2个一起才阻断)
func (m *Map) JudgeLineObstacleNew_GetOneEdgeObstacle(x1, y1, x2, y2 int) (bool, *Node) {
	//这一步 传入指针切片，内部把不是墙的节点添加完毕后传给外部
	var nowNode *Node
	//judgeNodeSlice := make([]*Node, 0, 20)
	//1.直线同1行
	if x1 == x2 {
		if y1 < y2 {
			//1start  2
			for newY := y1 + 1; newY < y2; newY++ {
				//如果直线障碍点有1个是墙 直接返回false
				nowNode = m.Nodes[x1][newY]
				if nowNode.IsWall() {
					return false, nowNode //
				}
			}
		} else {
			//2 1start
			for newY := y1 - 1; newY > y2; newY-- {
				//如果直线障碍点有1个是墙 直接返回false
				nowNode = m.Nodes[x1][newY]
				if nowNode.IsWall() {
					return false, nowNode //
				}
			}
		}
		//都不是墙 代表可以相连
		return true, nil
	}
	//2.直线同1列
	if y1 == y2 {
		if x1 < x2 {
			//1 Small start
			//2 big
			for newX := x1 + 1; newX < x2; newX++ {
				//如果直线障碍点有1个是墙 直接返回false
				nowNode = m.Nodes[newX][y1]
				if nowNode.IsWall() {
					return false, nowNode //
				}
			}
		} else {
			//2 Small
			//1 big start
			for newX := x1 - 1; newX > x2; newX-- {
				//如果直线障碍点有1个是墙 直接返回false
				nowNode = m.Nodes[newX][y1]
				if nowNode.IsWall() {
					return false, nowNode //
				}
			}
		}
		//都不是墙 代表可以相连
		return true, nil
	}
	//0.先计算dx dy,用后面的点减去前面的点
	var nowNode1 *Node
	var nowNode2 *Node
	dx := x2 - x1
	dy := y2 - y1
	//3.处于标准的正方形对角线2点 判断是不是标准正方形 斜向45度 只选择必经点 只选择必经点 只选择必经点 只选择必经点!!!
	if dx == dy || dx == -dy {
		newX := x1
		newY := y1
		//
		if dx > 0 && dy > 0 {
			//1.打印右下斜
			//xy1 start
			//    \
			//      xy2终点
			for {
				nowNode1 = m.Nodes[newX][newY+1]
				nowNode2 = m.Nodes[newX+1][newY]
				//1.右和下 擦边点2个都是墙 才不能相连
				if nowNode1.IsWall() && nowNode2.IsWall() {
					return false, nowNode1 //
				}
				//2.再改变当前点索引
				newX++
				newY++
				if newX == x2 && newY == y2 {
					return true, nil //到终点了 代表可以相连
				}
				//必经点是墙
				nowNode = m.Nodes[newX][newY]
				if nowNode.IsWall() {
					return false, nowNode //
				}
			}
		} else if dx > 0 && dy < 0 {
			//2.打印左下斜
			//      xy1 start
			//    /
			//xy2终点
			for {
				nowNode1 = m.Nodes[newX][newY-1]
				nowNode2 = m.Nodes[newX+1][newY]
				//1.左和下 擦边点2个都是墙
				if nowNode1.IsWall() && nowNode2.IsWall() {
					return false, nowNode1 //
				}
				//2.再改变当前点索引
				newX++
				newY--
				if newX == x2 && newY == y2 {
					return true, nil //到终点了 代表可以相连
				}
				//必经点是墙
				nowNode = m.Nodes[newX][newY]
				if nowNode.IsWall() {
					return false, nowNode //
				}
			}
		} else if dx < 0 && dy < 0 {
			//3.打印左上斜
			//xy2
			//    \
			//      xy1 start
			for {
				nowNode1 = m.Nodes[newX-1][newY]
				nowNode2 = m.Nodes[newX][newY-1]
				//1.上和左 擦边点2个都是墙
				if nowNode1.IsWall() && nowNode2.IsWall() {
					return false, nowNode1 //
				}
				//2.再改变当前点索引
				newX--
				newY--
				if newX == x2 && newY == y2 {
					return true, nil //到终点了 代表可以相连
				}
				//必经点是墙
				nowNode = m.Nodes[newX][newY]
				if nowNode.IsWall() {
					return false, nowNode //
				}
			}
		} else if dx < 0 && dy > 0 {
			//4.打印右上斜
			//      xy2
			//    /
			//xy1
			for {
				nowNode1 = m.Nodes[newX-1][newY]
				nowNode2 = m.Nodes[newX][newY+1]
				//1.上和右 擦边点2个都是墙
				if nowNode1.IsWall() && nowNode2.IsWall() {
					return false, nowNode1 //
				}
				//2.再改变当前点索引
				newX--
				newY++
				if newX == x2 && newY == y2 {
					return true, nil //到终点了 代表可以相连
				}
				//已知必经点不是墙
				//必经点是墙
				nowNode = m.Nodes[newX][newY]
				if nowNode.IsWall() {
					return false, nowNode //
				}
			}
		}
	}
	//0.前面已经计算
	//dx := x2 - x1
	//dy := y2 - y1
	//4.y = kx +b 的情况
	//fmt.Println("处于y = kx +b的情况")
	//1.计算 k 和 b
	k, b := 0.0, 0.0
	k = float64(dy) / float64(dx)
	b = float64(y1) + 0.5 - (k * (float64(x1) + 0.5))
	isYKXBRightUp := false
	isYKXBRightDown := false
	isYKXBLeftDown := false
	isYKXBLeftUp := false
	//判断方向
	if dx > 0 && dy > 0 {
		//1.打印右下斜
		//xy1 start
		//    \
		//      xy2终点
		//函数图右上
		isYKXBRightUp = true
	} else if dx > 0 && dy < 0 {
		//2.打印左下斜
		//      xy1 start
		//    /
		//xy2终点
		//函数图右下
		isYKXBRightDown = true
	} else if dx < 0 && dy < 0 {
		//3.打印左上斜
		//xy2
		//    \
		//      xy1 start
		//函数图左下
		isYKXBLeftDown = true
	} else if dx < 0 && dy > 0 {
		//4.打印右上斜
		//      xy2
		//    /
		//xy1
		//函数图左上
		isYKXBLeftUp = true
	}
	//
	smallX, smallY, bigX, bigY := 0, 0, 0, 0
	//判断选择哪个轴
	if absInt(dx) > absInt(dy) {
		//选择x轴作为变量 动态Y判断左右
		focusIntYUp := 0
		focusButUp := false
		focusButDown := false
		dynamicY := 0.0
		intDynamicY := 0
		//判断函数图方向中,谁的索引差大就选择索引轴(X或Y轴)作为遍历去计算另一个值
		if isYKXBRightUp {
			//函数图右上
			//      xy2
			//    /
			//xy1 start
			smallX = x1
			bigX = x2
			//通用遍历左右
			for XRight := smallX + 1; XRight <= bigX; XRight++ {
				//0.重置当前Y与交点上下判断
				focusButUp = false
				focusButDown = false
				//1.根据不断增加的X获得Y
				dynamicY = (k * float64(XRight)) + b
				intDynamicY = int(dynamicY)
				//2.计算当前Y与当前Y的floor差值
				//3.特殊情况:刚好处于交点 也就是Floor和Ceil相等
				if math.Floor(dynamicY) == math.Ceil(dynamicY) {
					focusButUp, focusButDown = true, true
					focusIntYUp = intDynamicY //当前上Y索引 去掉小数
				} else if dynamicY-math.Floor(dynamicY) < 0.01 { //focusDis必定大于0
					focusButUp = true         //当前Y值在交点上方
					focusIntYUp = intDynamicY //当前上Y索引
				} else if math.Ceil(dynamicY)-dynamicY <= 0.01 { //focusDis必定大于0
					focusButDown = true           //当前Y值在交点下方
					focusIntYUp = intDynamicY + 1 //当前上Y索引
				}
				//3.如果差值小于某个数 认为碰到交点
				if focusButUp || focusButDown {
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[XRight-1][focusIntYUp-1] //中点左下
					if nowNode.IsWall() {
						return false, nowNode
					}
					//擦边点 连通性未确认不是墙
					nowNode1 = m.Nodes[XRight-1][focusIntYUp]
					nowNode2 = m.Nodes[XRight][focusIntYUp-1]
					if nowNode1.IsWall() && nowNode2.IsWall() { //中点左上 //中点右下
						return false, nowNode1
					}
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight][focusIntYUp] //中点右上
					if nowNode.IsWall() {
						return false, nowNode
					}
				} else {
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight-1][intDynamicY] //左
					if nowNode.IsWall() {
						return false, nowNode
					}
					nowNode = m.Nodes[XRight][intDynamicY] //右
					if nowNode.IsWall() {
						return false, nowNode
					}
				}
			}
		} else if isYKXBRightDown {
			//函数图右下
			//xy1 start
			//    \
			//      xy2
			smallX = x1
			bigX = x2
			//通用遍历左右
			for XRight := smallX + 1; XRight <= bigX; XRight++ {
				//0.重置当前Y与交点上下判断
				focusButUp = false
				focusButDown = false
				//1.根据不断增加的X获得Y
				dynamicY = (k * float64(XRight)) + b
				intDynamicY = int(dynamicY)
				//2.计算当前Y与当前Y的floor差值
				//3.特殊情况:刚好处于交点 也就是Floor和Ceil相等
				if math.Floor(dynamicY) == math.Ceil(dynamicY) {
					focusButUp, focusButDown = true, true
					focusIntYUp = intDynamicY //当前上Y索引 去掉小数
				} else if dynamicY-math.Floor(dynamicY) < 0.01 { //focusDis必定大于0
					focusButUp = true         //当前Y值在交点上方
					focusIntYUp = intDynamicY //当前上Y索引
				} else if math.Ceil(dynamicY)-dynamicY <= 0.01 { //focusDis必定大于0
					focusButDown = true           //当前Y值在交点下方
					focusIntYUp = intDynamicY + 1 //当前上Y索引
				}
				//3.如果差值小于某个数 认为碰到交点
				if focusButUp || focusButDown {
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[XRight-1][focusIntYUp] //中点左上
					if nowNode.IsWall() {
						return false, nowNode
					}
					//擦边点 连通性未确认不是墙
					nowNode1 = m.Nodes[XRight][focusIntYUp]
					nowNode2 = m.Nodes[XRight-1][focusIntYUp-1]
					if nowNode1.IsWall() && nowNode2.IsWall() { //中点右上 //中点左下
						return false, nowNode1
					}
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight][focusIntYUp-1] //中点右下
					if nowNode.IsWall() {
						return false, nowNode
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight-1][intDynamicY] //左
					if nowNode.IsWall() {
						return false, nowNode
					}
					nowNode = m.Nodes[XRight][intDynamicY] //右
					if nowNode.IsWall() {
						return false, nowNode
					}
				}
			}
		} else if isYKXBLeftDown {
			//函数图左下
			//     xy1 start
			//    /
			//xy2
			smallX = x2
			bigX = x1
			//通用遍历左右
			for XRight := bigX; XRight > smallX; XRight-- {
				//0.重置当前Y与交点上下判断
				focusButUp = false
				focusButDown = false
				//1.根据不断增加的X获得Y
				dynamicY = (k * float64(XRight)) + b
				intDynamicY = int(dynamicY)
				//2.计算当前Y与当前Y的floor差值
				//3.特殊情况:刚好处于交点 也就是Floor和Ceil相等
				if math.Floor(dynamicY) == math.Ceil(dynamicY) {
					focusButUp, focusButDown = true, true
					focusIntYUp = intDynamicY //当前上Y索引 去掉小数
				} else if dynamicY-math.Floor(dynamicY) < 0.01 { //focusDis必定大于0
					focusButUp = true         //当前Y值在交点上方
					focusIntYUp = intDynamicY //当前上Y索引
				} else if math.Ceil(dynamicY)-dynamicY <= 0.01 { //focusDis必定大于0
					focusButDown = true           //当前Y值在交点下方
					focusIntYUp = intDynamicY + 1 //当前上Y索引
				}
				//3.如果差值小于某个数 认为碰到交点
				if focusButUp || focusButDown {
					//4.判断必经点 擦边点 注意：不同方向必经点不一样
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][focusIntYUp-1], m.Nodes[XRight][focusIntYUp],
					//	m.Nodes[XRight-1][focusIntYUp], m.Nodes[XRight][focusIntYUp-1])
					//5.如果当前方向是 /
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight][focusIntYUp] //中点右上
					if nowNode.IsWall() {
						return false, nowNode
					}
					//擦边点 连通性未确认不是墙
					nowNode1 = m.Nodes[XRight-1][focusIntYUp]
					nowNode2 = m.Nodes[XRight][focusIntYUp-1]
					if nowNode1.IsWall() && nowNode2.IsWall() { //中点左上 //中点右下
						return false, nowNode1
					}
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[XRight-1][focusIntYUp-1] //中点左下
					if nowNode.IsWall() {
						return false, nowNode
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight][intDynamicY] //右
					if nowNode.IsWall() {
						return false, nowNode
					}
					nowNode = m.Nodes[XRight-1][intDynamicY] //左
					if nowNode.IsWall() {
						return false, nowNode
					}
				}
			}
		} else if isYKXBLeftUp {
			//函数图左上
			//xy2
			//    \
			//    xy1 start
			smallX = x2
			bigX = x1
			//通用遍历左右
			for XRight := bigX; XRight > smallX; XRight-- {
				//0.重置当前Y与交点上下判断
				focusButUp = false
				focusButDown = false
				//1.根据不断增加的X获得Y
				dynamicY = (k * float64(XRight)) + b
				intDynamicY = int(dynamicY)
				//2.计算当前Y与当前Y的floor差值
				//3.特殊情况:刚好处于交点 也就是Floor和Ceil相等
				if math.Floor(dynamicY) == math.Ceil(dynamicY) {
					focusButUp, focusButDown = true, true
					focusIntYUp = intDynamicY //当前上Y索引 去掉小数
				} else if dynamicY-math.Floor(dynamicY) < 0.01 { //focusDis必定大于0
					focusButUp = true         //当前Y值在交点上方
					focusIntYUp = intDynamicY //当前上Y索引
				} else if math.Ceil(dynamicY)-dynamicY <= 0.01 { //focusDis必定大于0
					focusButDown = true           //当前Y值在交点下方
					focusIntYUp = intDynamicY + 1 //当前上Y索引
				}
				//3.如果差值小于某个数 认为碰到交点
				if focusButUp || focusButDown {
					//4.判断必经点 擦边点 注意：不同方向必经点不一样
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][focusIntYUp-1], m.Nodes[XRight][focusIntYUp],
					//	m.Nodes[XRight-1][focusIntYUp], m.Nodes[XRight][focusIntYUp-1])
					//5.如果当前方向是 /
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight][focusIntYUp-1] //中点右下
					if nowNode.IsWall() {
						return false, nowNode
					}
					//擦边点 连通性未确认不是墙
					nowNode1 = m.Nodes[XRight-1][focusIntYUp-1]
					nowNode2 = m.Nodes[XRight][focusIntYUp]
					if nowNode1.IsWall() && nowNode2.IsWall() { //中点左下  //中点右上
						return false, nowNode1
					}
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[XRight-1][focusIntYUp] //中点左上
					if nowNode.IsWall() {
						return false, nowNode
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight][intDynamicY] //右
					if nowNode.IsWall() {
						return false, nowNode
					}
					nowNode = m.Nodes[XRight-1][intDynamicY] //左
					if nowNode.IsWall() {
						return false, nowNode
					}
				}
			}
		}
	} else { //dx < dy
		//选择y轴作为变量 判断上下
		focusIntXLeft := 0
		focusButLeft := false
		focusButRight := false
		dynamicX := 0.0
		intDynamicX := 0
		//判断函数图方向中,谁的索引差大就选择索引轴(X或Y轴)作为遍历去计算另一个值
		if isYKXBRightUp {
			//函数图右上
			//      xy2
			//    /
			//xy1 start
			smallY = y1
			bigY = y2
			//通用遍历上下
			for YUp := smallY + 1; YUp <= bigY; YUp++ {
				//0.重置当前Y与交点上下判断
				focusButLeft = false
				focusButRight = false
				//1.根据不断增加的Y获得X
				dynamicX = (float64(YUp) - b) / k
				intDynamicX = int(dynamicX)
				//2.计算当前X与当前X的floor差值
				if math.Ceil(dynamicX) == math.Floor(dynamicX) {
					focusButLeft, focusButRight = true, true
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				} else if math.Ceil(dynamicX)-dynamicX < 0.01 { //focusDis必定大于0
					focusButLeft = true         //当前X值在交点左
					focusIntXLeft = intDynamicX //当前左X索引
				} else if dynamicX-math.Floor(dynamicX) < 0.01 { //focusDis必定大于0
					focusButRight = true            //当前X值在交点右
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				}
				//3.如果差值小于某个数或相等 认为碰到交点
				if focusButLeft || focusButRight {
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft][YUp-1] //中点左下
					if nowNode.IsWall() {
						return false, nowNode
					}
					//擦边点 连通性未确认不是墙
					nowNode1 = m.Nodes[focusIntXLeft+1][YUp-1]
					nowNode2 = m.Nodes[focusIntXLeft][YUp]
					if nowNode1.IsWall() && nowNode2.IsWall() { //中点右下 //中点左上
						return false, nowNode1
					}
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft+1][YUp] //中点右上
					if nowNode.IsWall() {
						return false, nowNode
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[intDynamicX][YUp-1] //下
					if nowNode.IsWall() {
						return false, nowNode
					}
					nowNode = m.Nodes[intDynamicX][YUp] //上
					if nowNode.IsWall() {
						return false, nowNode
					}
				}
			}
		} else if isYKXBRightDown {
			//函数图右下
			//xy1 start
			//    \
			//      xy2
			smallY = y2
			bigY = y1
			//通用遍历上下
			for YUp := bigY; YUp > smallY; YUp-- {
				//0.重置当前Y与交点上下判断
				focusButLeft = false
				focusButRight = false
				//1.根据不断增加的Y获得X
				dynamicX = (float64(YUp) - b) / k
				intDynamicX = int(dynamicX)
				//2.计算当前X与当前X的floor差值
				if math.Ceil(dynamicX) == math.Floor(dynamicX) {
					focusButLeft, focusButRight = true, true
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				} else if math.Ceil(dynamicX)-dynamicX < 0.01 { //focusDis必定大于0
					focusButLeft = true         //当前X值在交点左
					focusIntXLeft = intDynamicX //当前左X索引
				} else if dynamicX-math.Floor(dynamicX) < 0.01 { //focusDis必定大于0
					focusButRight = true            //当前X值在交点右
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				}
				//3.如果差值小于某个数或相等 认为碰到交点
				if focusButLeft || focusButRight {
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft][YUp] //中点左上
					if nowNode.IsWall() {
						return false, nowNode
					}
					//擦边点 连通性未确认不是墙
					nowNode1 = m.Nodes[focusIntXLeft+1][YUp]
					nowNode2 = m.Nodes[focusIntXLeft][YUp-1]
					if m.Nodes[focusIntXLeft+1][YUp].IsWall() && m.Nodes[focusIntXLeft][YUp-1].IsWall() { //中点右上  //中点左下
						return false, nowNode1
					}
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft+1][YUp-1] //中点右下
					if nowNode.IsWall() {
						return false, nowNode
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[intDynamicX][YUp] //上
					if nowNode.IsWall() {
						return false, nowNode
					}
					nowNode = m.Nodes[intDynamicX][YUp-1] //下
					if nowNode.IsWall() {
						return false, nowNode
					}
				}
			}
		} else if isYKXBLeftDown {
			//函数图左下
			//     xy1 start
			//    /
			//xy2终点
			smallY = y2
			bigY = y1
			//通用遍历上下
			for YUp := bigY; YUp > smallY; YUp-- {
				//0.重置当前Y与交点上下判断
				focusButLeft = false
				focusButRight = false
				//1.根据不断增加的Y获得X
				dynamicX = (float64(YUp) - b) / k
				intDynamicX = int(dynamicX)
				//2.计算当前X与当前X的floor差值
				if math.Ceil(dynamicX) == math.Floor(dynamicX) {
					focusButLeft, focusButRight = true, true
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				} else if math.Ceil(dynamicX)-dynamicX < 0.01 { //focusDis必定大于0
					focusButLeft = true         //当前X值在交点左
					focusIntXLeft = intDynamicX //当前左X索引
				} else if dynamicX-math.Floor(dynamicX) < 0.01 { //focusDis必定大于0
					focusButRight = true            //当前X值在交点右
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				}
				//3.如果差值小于某个数或相等 认为碰到交点
				if focusButLeft || focusButRight {
					//4.判断必经点 擦边点 注意：不同方向必经点不一样
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][focusIntYUp-1], m.Nodes[XRight][focusIntYUp],
					//	m.Nodes[XRight-1][focusIntYUp], m.Nodes[XRight][focusIntYUp-1])
					//5.如果当前方向是 /
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft+1][YUp] //中点右上
					if nowNode.IsWall() {
						return false, nowNode
					}
					//擦边点 连通性未确认不是墙
					nowNode1 = m.Nodes[focusIntXLeft][YUp]
					nowNode2 = m.Nodes[focusIntXLeft+1][YUp-1]
					if m.Nodes[focusIntXLeft][YUp].IsWall() && m.Nodes[focusIntXLeft+1][YUp-1].IsWall() { //中点左上 //中点右下
						return false, nowNode1
					}
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft][YUp-1] //中点左下
					if nowNode.IsWall() {
						return false, nowNode
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[intDynamicX][YUp] //上
					if nowNode.IsWall() {
						return false, nowNode
					}
					nowNode = m.Nodes[intDynamicX][YUp-1] //下
					if nowNode.IsWall() {
						return false, nowNode
					}
				}
			}
		} else if isYKXBLeftUp {
			//函数图左上
			//xy2
			//    \
			//    xy1 start
			smallY = y1
			bigY = y2
			//通用遍历上下
			for YUp := smallY + 1; YUp <= bigY; YUp++ {
				//0.重置当前Y与交点上下判断
				focusButLeft = false
				focusButRight = false
				//1.根据不断增加的Y获得X
				dynamicX = (float64(YUp) - b) / k
				intDynamicX = int(dynamicX)
				//2.计算当前X与当前X的floor差值
				if math.Ceil(dynamicX) == math.Floor(dynamicX) {
					focusButLeft, focusButRight = true, true
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				} else if math.Ceil(dynamicX)-dynamicX < 0.01 { //focusDis必定大于0
					focusButLeft = true         //当前X值在交点左
					focusIntXLeft = intDynamicX //当前左X索引
				} else if dynamicX-math.Floor(dynamicX) < 0.01 { //focusDis必定大于0
					focusButRight = true            //当前X值在交点右
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				}
				//3.如果差值小于某个数或相等 认为碰到交点
				if focusButLeft || focusButRight {
					//4.判断必经点 擦边点 注意：不同方向必经点不一样
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][focusIntYUp-1], m.Nodes[XRight][focusIntYUp],
					//	m.Nodes[XRight-1][focusIntYUp], m.Nodes[XRight][focusIntYUp-1])
					//5.如果当前方向是 /
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft+1][YUp-1] //中点右下
					if nowNode.IsWall() {
						return false, nowNode
					}
					//擦边点 连通性未确认不是墙
					nowNode1 = m.Nodes[focusIntXLeft+1][YUp]
					nowNode2 = m.Nodes[focusIntXLeft][YUp-1]
					if nowNode1.IsWall() && nowNode2.IsWall() { //中点右上 //中点左下
						return false, nowNode1
					}
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft][YUp] //中点左上
					if nowNode.IsWall() {
						return false, nowNode
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[intDynamicX][YUp-1] //下
					if nowNode.IsWall() {
						return false, nowNode
					}
					nowNode = m.Nodes[intDynamicX][YUp] //上
					if nowNode.IsWall() {
						return false, nowNode
					}
				}
			}
		}
	}
	//最终y=kx+b没有false则判定相连
	return true, nil
}

// JudgeLineObstacleNew_GetOneEdgeObstacle_UseBitMap 判断两点之间连线是否碰到障碍物 返回碰到的第1个障碍 默认参数x1, y1, x2, y2都在地图之内 version0.0.1:修改45度的双向判断(原来某个阻断 现在必须2个一起才阻断)
func (m *Map) JudgeLineObstacleNew_GetOneEdgeObstacle_UseBitMap(x1, y1, x2, y2 int) (bool, *Node) {
	//这一步 传入指针切片，内部把不是墙的节点添加完毕后传给外部
	//var nowNode *Node
	//judgeNodeSlice := make([]*Node, 0, 20)
	//1.直线同1行
	if x1 == x2 {
		if y1 < y2 {
			//1start  2
			for newY := y1 + 1; newY < y2; newY++ {
				//如果直线障碍点有1个是墙 直接返回false
				if m.isWall_DynamicRayPathFind(x1, newY) {
					return false, m.Nodes[x1][newY] //
				}
			}
		} else {
			//2 1start
			for newY := y1 - 1; newY > y2; newY-- {
				//如果直线障碍点有1个是墙 直接返回false
				if m.isWall_DynamicRayPathFind(x1, newY) {
					return false, m.Nodes[x1][newY] //
				}
			}
		}
		//都不是墙 代表可以相连
		return true, nil
	}
	//2.直线同1列
	if y1 == y2 {
		if x1 < x2 {
			//1 Small start
			//2 big
			for newX := x1 + 1; newX < x2; newX++ {
				//如果直线障碍点有1个是墙 直接返回false
				if m.isWall_DynamicRayPathFind(newX, y1) {
					return false, m.Nodes[newX][y1] //
				}
			}
		} else {
			//2 Small
			//1 big start
			for newX := x1 - 1; newX > x2; newX-- {
				//如果直线障碍点有1个是墙 直接返回false
				if m.isWall_DynamicRayPathFind(newX, y1) {
					return false, m.Nodes[newX][y1] //
				}
			}
		}
		//都不是墙 代表可以相连
		return true, nil
	}
	//0.先计算dx dy,用后面的点减去前面的点
	//var nowNode1 *Node
	//var nowNode2 *Node
	dx := x2 - x1
	dy := y2 - y1
	//3.处于标准的正方形对角线2点 判断是不是标准正方形 斜向45度 只选择必经点 只选择必经点 只选择必经点 只选择必经点!!!
	if dx == dy || dx == -dy {
		newX := x1
		newY := y1
		//
		if dx > 0 && dy > 0 {
			//1.打印右下斜
			//xy1 start
			//    \
			//      xy2终点
			for {
				//nowNode1 = m.Nodes[newX][newY+1]
				//nowNode2 = m.Nodes[newX+1][newY]
				//1.右和下 擦边点2个都是墙 才不能相连
				if m.isWall_DynamicRayPathFind(newX, newY+1) && m.isWall_DynamicRayPathFind(newX+1, newY) {
					return false, m.Nodes[newX][newY+1] //
				}
				//2.再改变当前点索引
				newX++
				newY++
				if newX == x2 && newY == y2 {
					return true, nil //到终点了 代表可以相连
				}
				//必经点是墙
				//nowNode = m.Nodes[newX][newY]
				if m.isWall_DynamicRayPathFind(newX, newY) {
					return false, m.Nodes[newX][newY] //
				}
			}
		} else if dx > 0 && dy < 0 {
			//2.打印左下斜
			//      xy1 start
			//    /
			//xy2终点
			for {
				//nowNode1 = m.Nodes[newX][newY-1]
				//nowNode2 = m.Nodes[newX+1][newY]
				//1.左和下 擦边点2个都是墙
				if m.isWall_DynamicRayPathFind(newX, newY-1) && m.isWall_DynamicRayPathFind(newX+1, newY) {
					return false, m.Nodes[newX][newY-1] //
				}
				//2.再改变当前点索引
				newX++
				newY--
				if newX == x2 && newY == y2 {
					return true, nil //到终点了 代表可以相连
				}
				//必经点是墙
				//nowNode = m.Nodes[newX][newY]
				if m.isWall_DynamicRayPathFind(newX, newY) {
					return false, m.Nodes[newX][newY] //
				}
			}
		} else if dx < 0 && dy < 0 {
			//3.打印左上斜
			//xy2
			//    \
			//      xy1 start
			for {
				//nowNode1 = m.Nodes[newX-1][newY]
				//nowNode2 = m.Nodes[newX][newY-1]
				//1.上和左 擦边点2个都是墙
				if m.isWall_DynamicRayPathFind(newX-1, newY) && m.isWall_DynamicRayPathFind(newX, newY-1) {
					return false, m.Nodes[newX-1][newY] //
				}
				//2.再改变当前点索引
				newX--
				newY--
				if newX == x2 && newY == y2 {
					return true, nil //到终点了 代表可以相连
				}
				//必经点是墙
				//nowNode = m.Nodes[newX][newY]
				if m.isWall_DynamicRayPathFind(newX, newY) {
					return false, m.Nodes[newX][newY] //
				}
			}
		} else if dx < 0 && dy > 0 {
			//4.打印右上斜
			//      xy2
			//    /
			//xy1
			for {
				//nowNode1 = m.Nodes[newX-1][newY]
				//nowNode2 = m.Nodes[newX][newY+1]
				//1.上和右 擦边点2个都是墙
				if m.isWall_DynamicRayPathFind(newX-1, newY) && m.isWall_DynamicRayPathFind(newX, newY+1) {
					return false, m.Nodes[newX-1][newY] //
				}
				//2.再改变当前点索引
				newX--
				newY++
				if newX == x2 && newY == y2 {
					return true, nil //到终点了 代表可以相连
				}
				//已知必经点不是墙
				//必经点是墙
				//nowNode = m.Nodes[newX][newY]
				if m.isWall_DynamicRayPathFind(newX, newY) {
					return false, m.Nodes[newX][newY] //
				}
			}
		}
	}
	//0.前面已经计算
	//dx := x2 - x1
	//dy := y2 - y1
	//4.y = kx +b 的情况
	//fmt.Println("处于y = kx +b的情况")
	//1.计算 k 和 b
	k, b := 0.0, 0.0
	k = float64(dy) / float64(dx)
	b = float64(y1) + 0.5 - (k * (float64(x1) + 0.5))
	isYKXBRightUp := false
	isYKXBRightDown := false
	isYKXBLeftDown := false
	isYKXBLeftUp := false
	//判断方向
	if dx > 0 && dy > 0 {
		//1.打印右下斜
		//xy1 start
		//    \
		//      xy2终点
		//函数图右上
		isYKXBRightUp = true
	} else if dx > 0 && dy < 0 {
		//2.打印左下斜
		//      xy1 start
		//    /
		//xy2终点
		//函数图右下
		isYKXBRightDown = true
	} else if dx < 0 && dy < 0 {
		//3.打印左上斜
		//xy2
		//    \
		//      xy1 start
		//函数图左下
		isYKXBLeftDown = true
	} else if dx < 0 && dy > 0 {
		//4.打印右上斜
		//      xy2
		//    /
		//xy1
		//函数图左上
		isYKXBLeftUp = true
	}
	//
	smallX, smallY, bigX, bigY := 0, 0, 0, 0
	//判断选择哪个轴
	if absInt(dx) > absInt(dy) {
		//选择x轴作为变量 动态Y判断左右
		focusIntYUp := 0
		focusButUp := false
		focusButDown := false
		dynamicY := 0.0
		intDynamicY := 0
		//判断函数图方向中,谁的索引差大就选择索引轴(X或Y轴)作为遍历去计算另一个值
		if isYKXBRightUp {
			//函数图右上
			//      xy2
			//    /
			//xy1 start
			smallX = x1
			bigX = x2
			//通用遍历左右
			for XRight := smallX + 1; XRight <= bigX; XRight++ {
				//0.重置当前Y与交点上下判断
				focusButUp = false
				focusButDown = false
				//1.根据不断增加的X获得Y
				dynamicY = (k * float64(XRight)) + b
				intDynamicY = int(dynamicY)
				//2.计算当前Y与当前Y的floor差值
				//3.特殊情况:刚好处于交点 也就是Floor和Ceil相等
				if math.Floor(dynamicY) == math.Ceil(dynamicY) {
					focusButUp, focusButDown = true, true
					focusIntYUp = intDynamicY //当前上Y索引 去掉小数
				} else if dynamicY-math.Floor(dynamicY) <= epsilon { //focusDis必定大于0
					focusButUp = true         //当前Y值在交点上方
					focusIntYUp = intDynamicY //当前上Y索引
				} else if math.Ceil(dynamicY)-dynamicY <= epsilon { //focusDis必定大于0
					focusButDown = true           //当前Y值在交点下方
					focusIntYUp = intDynamicY + 1 //当前上Y索引
				}
				//3.如果差值小于某个数 认为碰到交点
				if focusButUp || focusButDown {
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					//nowNode = m.Nodes[XRight-1][focusIntYUp-1] //中点左下
					if m.isWall_DynamicRayPathFind(XRight-1, focusIntYUp-1) {
						return false, m.Nodes[XRight-1][focusIntYUp-1] //中点左下
					}
					//擦边点 连通性未确认不是墙
					//nowNode1 = m.Nodes[XRight-1][focusIntYUp]
					//nowNode2 = m.Nodes[XRight][focusIntYUp-1]
					if m.isWall_DynamicRayPathFind(XRight-1, focusIntYUp) && m.isWall_DynamicRayPathFind(XRight, focusIntYUp-1) { //中点左上 //中点右下
						return false, m.Nodes[XRight-1][focusIntYUp]
					}
					//必经点 连通性已确认不是墙
					//nowNode = m.Nodes[XRight][focusIntYUp] //中点右上
					if m.isWall_DynamicRayPathFind(XRight, focusIntYUp) {
						return false, m.Nodes[XRight][focusIntYUp] //中点右上
					}
				} else {
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					//nowNode = m.Nodes[XRight-1][intDynamicY] //左
					if m.isWall_DynamicRayPathFind(XRight-1, intDynamicY) {
						return false, m.Nodes[XRight-1][intDynamicY] //左
					}
					//nowNode = m.Nodes[XRight][intDynamicY] //右
					if m.isWall_DynamicRayPathFind(XRight, intDynamicY) {
						return false, m.Nodes[XRight][intDynamicY] //右
					}
				}
			}
		} else if isYKXBRightDown {
			//函数图右下
			//xy1 start
			//    \
			//      xy2
			smallX = x1
			bigX = x2
			//通用遍历左右
			for XRight := smallX + 1; XRight <= bigX; XRight++ {
				//0.重置当前Y与交点上下判断
				focusButUp = false
				focusButDown = false
				//1.根据不断增加的X获得Y
				dynamicY = (k * float64(XRight)) + b
				intDynamicY = int(dynamicY)
				//2.计算当前Y与当前Y的floor差值
				//3.特殊情况:刚好处于交点 也就是Floor和Ceil相等
				if math.Floor(dynamicY) == math.Ceil(dynamicY) {
					focusButUp, focusButDown = true, true
					focusIntYUp = intDynamicY //当前上Y索引 去掉小数
				} else if dynamicY-math.Floor(dynamicY) <= epsilon { //focusDis必定大于0
					focusButUp = true         //当前Y值在交点上方
					focusIntYUp = intDynamicY //当前上Y索引
				} else if math.Ceil(dynamicY)-dynamicY <= epsilon { //focusDis必定大于0
					focusButDown = true           //当前Y值在交点下方
					focusIntYUp = intDynamicY + 1 //当前上Y索引
				}
				//3.如果差值小于某个数 认为碰到交点
				if focusButUp || focusButDown {
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					//nowNode = m.Nodes[XRight-1][focusIntYUp] //中点左上
					if m.isWall_DynamicRayPathFind(XRight-1, focusIntYUp) {
						return false, m.Nodes[XRight-1][focusIntYUp] //中点左上
					}
					//擦边点 连通性未确认不是墙
					//nowNode1 = m.Nodes[XRight][focusIntYUp]
					//nowNode2 = m.Nodes[XRight-1][focusIntYUp-1]
					if m.isWall_DynamicRayPathFind(XRight, focusIntYUp) && m.isWall_DynamicRayPathFind(XRight-1, focusIntYUp-1) { //中点右上 //中点左下
						return false, m.Nodes[XRight][focusIntYUp]
					}
					//必经点 连通性已确认不是墙
					//nowNode = m.Nodes[XRight][focusIntYUp-1] //中点右下
					if m.isWall_DynamicRayPathFind(XRight, focusIntYUp-1) {
						return false, m.Nodes[XRight][focusIntYUp-1] //中点右下
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					//nowNode = m.Nodes[XRight-1][intDynamicY] //左
					if m.isWall_DynamicRayPathFind(XRight-1, intDynamicY) {
						return false, m.Nodes[XRight-1][intDynamicY] //左
					}
					//nowNode = m.Nodes[XRight][intDynamicY] //右
					if m.isWall_DynamicRayPathFind(XRight, intDynamicY) {
						return false, m.Nodes[XRight][intDynamicY] //右
					}
				}
			}
		} else if isYKXBLeftDown {
			//函数图左下
			//     xy1 start
			//    /
			//xy2
			smallX = x2
			bigX = x1
			//通用遍历左右
			for XRight := bigX; XRight > smallX; XRight-- {
				//0.重置当前Y与交点上下判断
				focusButUp = false
				focusButDown = false
				//1.根据不断增加的X获得Y
				dynamicY = (k * float64(XRight)) + b
				intDynamicY = int(dynamicY)
				//2.计算当前Y与当前Y的floor差值
				//3.特殊情况:刚好处于交点 也就是Floor和Ceil相等
				if math.Floor(dynamicY) == math.Ceil(dynamicY) {
					focusButUp, focusButDown = true, true
					focusIntYUp = intDynamicY //当前上Y索引 去掉小数
				} else if dynamicY-math.Floor(dynamicY) <= epsilon { //focusDis必定大于0
					focusButUp = true         //当前Y值在交点上方
					focusIntYUp = intDynamicY //当前上Y索引
				} else if math.Ceil(dynamicY)-dynamicY <= epsilon { //focusDis必定大于0
					focusButDown = true           //当前Y值在交点下方
					focusIntYUp = intDynamicY + 1 //当前上Y索引
				}
				//3.如果差值小于某个数 认为碰到交点
				if focusButUp || focusButDown {
					//4.判断必经点 擦边点 注意：不同方向必经点不一样
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][focusIntYUp-1], m.Nodes[XRight][focusIntYUp],
					//	m.Nodes[XRight-1][focusIntYUp], m.Nodes[XRight][focusIntYUp-1])
					//5.如果当前方向是 /
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点 连通性已确认不是墙
					//nowNode = m.Nodes[XRight][focusIntYUp] //中点右上
					if m.isWall_DynamicRayPathFind(XRight, focusIntYUp) {
						return false, m.Nodes[XRight][focusIntYUp] //中点右上
					}
					//擦边点 连通性未确认不是墙
					//nowNode1 = m.Nodes[XRight-1][focusIntYUp]
					//nowNode2 = m.Nodes[XRight][focusIntYUp-1]
					if m.isWall_DynamicRayPathFind(XRight-1, focusIntYUp) && m.isWall_DynamicRayPathFind(XRight, focusIntYUp-1) { //中点左上 //中点右下
						return false, m.Nodes[XRight-1][focusIntYUp]
					}
					//必经点  连通性已确认不是墙
					//nowNode = m.Nodes[XRight-1][focusIntYUp-1] //中点左下
					if m.isWall_DynamicRayPathFind(XRight-1, focusIntYUp-1) {
						return false, m.Nodes[XRight-1][focusIntYUp-1] //中点左下
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					//nowNode = m.Nodes[XRight][intDynamicY] //右
					if m.isWall_DynamicRayPathFind(XRight, intDynamicY) {
						return false, m.Nodes[XRight][intDynamicY] //右
					}
					//nowNode = m.Nodes[XRight-1][intDynamicY] //左
					if m.isWall_DynamicRayPathFind(XRight-1, intDynamicY) {
						return false, m.Nodes[XRight-1][intDynamicY] //左
					}
				}
			}
		} else if isYKXBLeftUp {
			//函数图左上
			//xy2
			//    \
			//    xy1 start
			smallX = x2
			bigX = x1
			//通用遍历左右
			for XRight := bigX; XRight > smallX; XRight-- {
				//0.重置当前Y与交点上下判断
				focusButUp = false
				focusButDown = false
				//1.根据不断增加的X获得Y
				dynamicY = (k * float64(XRight)) + b
				intDynamicY = int(dynamicY)
				//2.计算当前Y与当前Y的floor差值
				//3.特殊情况:刚好处于交点 也就是Floor和Ceil相等
				if math.Floor(dynamicY) == math.Ceil(dynamicY) {
					focusButUp, focusButDown = true, true
					focusIntYUp = intDynamicY //当前上Y索引 去掉小数
				} else if dynamicY-math.Floor(dynamicY) <= epsilon { //focusDis必定大于0
					focusButUp = true         //当前Y值在交点上方
					focusIntYUp = intDynamicY //当前上Y索引
				} else if math.Ceil(dynamicY)-dynamicY <= epsilon { //focusDis必定大于0
					focusButDown = true           //当前Y值在交点下方
					focusIntYUp = intDynamicY + 1 //当前上Y索引
				}
				//3.如果差值小于某个数 认为碰到交点
				if focusButUp || focusButDown {
					//4.判断必经点 擦边点 注意：不同方向必经点不一样
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][focusIntYUp-1], m.Nodes[XRight][focusIntYUp],
					//	m.Nodes[XRight-1][focusIntYUp], m.Nodes[XRight][focusIntYUp-1])
					//5.如果当前方向是 /
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点 连通性已确认不是墙
					//nowNode = m.Nodes[XRight][focusIntYUp-1] //中点右下
					if m.isWall_DynamicRayPathFind(XRight, focusIntYUp-1) {
						return false, m.Nodes[XRight][focusIntYUp-1] //中点右下
					}
					//擦边点 连通性未确认不是墙
					//nowNode1 = m.Nodes[XRight-1][focusIntYUp-1]
					//nowNode2 = m.Nodes[XRight][focusIntYUp]
					if m.isWall_DynamicRayPathFind(XRight-1, focusIntYUp-1) && m.isWall_DynamicRayPathFind(XRight, focusIntYUp) { //中点左下  //中点右上
						return false, m.Nodes[XRight-1][focusIntYUp-1]
					}
					//必经点  连通性已确认不是墙
					//nowNode = m.Nodes[XRight-1][focusIntYUp] //中点左上
					if m.isWall_DynamicRayPathFind(XRight-1, focusIntYUp) {
						return false, m.Nodes[XRight-1][focusIntYUp] //中点左上
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					//nowNode = m.Nodes[XRight][intDynamicY] //右
					if m.isWall_DynamicRayPathFind(XRight, intDynamicY) {
						return false, m.Nodes[XRight][intDynamicY] //右
					}
					//nowNode = m.Nodes[XRight-1][intDynamicY] //左
					if m.isWall_DynamicRayPathFind(XRight-1, intDynamicY) {
						return false, m.Nodes[XRight-1][intDynamicY] //左
					}
				}
			}
		}
	} else { //dx < dy
		//选择y轴作为变量 判断上下
		focusIntXLeft := 0
		focusButLeft := false
		focusButRight := false
		dynamicX := 0.0
		intDynamicX := 0
		//判断函数图方向中,谁的索引差大就选择索引轴(X或Y轴)作为遍历去计算另一个值
		if isYKXBRightUp {
			//函数图右上
			//      xy2
			//    /
			//xy1 start
			smallY = y1
			bigY = y2
			//通用遍历上下
			for YUp := smallY + 1; YUp <= bigY; YUp++ {
				//0.重置当前Y与交点上下判断
				focusButLeft = false
				focusButRight = false
				//1.根据不断增加的Y获得X
				dynamicX = (float64(YUp) - b) / k
				intDynamicX = int(dynamicX)
				//2.计算当前X与当前X的floor差值
				if math.Ceil(dynamicX) == math.Floor(dynamicX) {
					focusButLeft, focusButRight = true, true
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				} else if math.Ceil(dynamicX)-dynamicX <= epsilon { //focusDis必定大于0
					focusButLeft = true         //当前X值在交点左
					focusIntXLeft = intDynamicX //当前左X索引
				} else if dynamicX-math.Floor(dynamicX) <= epsilon { //focusDis必定大于0
					focusButRight = true            //当前X值在交点右
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				}
				//3.如果差值小于某个数或相等 认为碰到交点
				if focusButLeft || focusButRight {
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					//nowNode = m.Nodes[focusIntXLeft][YUp-1] //中点左下
					if m.isWall_DynamicRayPathFind(focusIntXLeft, YUp-1) {
						return false, m.Nodes[focusIntXLeft][YUp-1] //中点左下
					}
					//擦边点 连通性未确认不是墙
					//nowNode1 = m.Nodes[focusIntXLeft+1][YUp-1]
					//nowNode2 = m.Nodes[focusIntXLeft][YUp]
					if m.isWall_DynamicRayPathFind(focusIntXLeft+1, YUp-1) && m.isWall_DynamicRayPathFind(focusIntXLeft, YUp) { //中点右下 //中点左上
						return false, m.Nodes[focusIntXLeft+1][YUp-1]
					}
					//必经点 连通性已确认不是墙
					//nowNode = m.Nodes[focusIntXLeft+1][YUp] //中点右上
					if m.isWall_DynamicRayPathFind(focusIntXLeft+1, YUp) {
						return false, m.Nodes[focusIntXLeft+1][YUp] //中点右上
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					//nowNode = m.Nodes[intDynamicX][YUp-1] //下
					if m.isWall_DynamicRayPathFind(intDynamicX, YUp-1) {
						return false, m.Nodes[intDynamicX][YUp-1] //下
					}
					//nowNode = m.Nodes[intDynamicX][YUp] //上
					if m.isWall_DynamicRayPathFind(intDynamicX, YUp) {
						return false, m.Nodes[intDynamicX][YUp] //上
					}
				}
			}
		} else if isYKXBRightDown {
			//函数图右下
			//xy1 start
			//    \
			//      xy2
			smallY = y2
			bigY = y1
			//通用遍历上下
			for YUp := bigY; YUp > smallY; YUp-- {
				//0.重置当前Y与交点上下判断
				focusButLeft = false
				focusButRight = false
				//1.根据不断增加的Y获得X
				dynamicX = (float64(YUp) - b) / k
				intDynamicX = int(dynamicX)
				//2.计算当前X与当前X的floor差值
				if math.Ceil(dynamicX) == math.Floor(dynamicX) {
					focusButLeft, focusButRight = true, true
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				} else if math.Ceil(dynamicX)-dynamicX <= epsilon { //focusDis必定大于0
					focusButLeft = true         //当前X值在交点左
					focusIntXLeft = intDynamicX //当前左X索引
				} else if dynamicX-math.Floor(dynamicX) <= epsilon { //focusDis必定大于0
					focusButRight = true            //当前X值在交点右
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				}
				//3.如果差值小于某个数或相等 认为碰到交点
				if focusButLeft || focusButRight {
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					//nowNode = m.Nodes[focusIntXLeft][YUp] //中点左上
					if m.isWall_DynamicRayPathFind(focusIntXLeft, YUp) {
						return false, m.Nodes[focusIntXLeft][YUp] //中点左上
					}
					//擦边点 连通性未确认不是墙
					//nowNode1 = m.Nodes[focusIntXLeft+1][YUp]
					//nowNode2 = m.Nodes[focusIntXLeft][YUp-1]
					if m.isWall_DynamicRayPathFind(focusIntXLeft+1, YUp) && m.isWall_DynamicRayPathFind(focusIntXLeft, YUp-1) { //中点右上  //中点左下
						return false, m.Nodes[focusIntXLeft+1][YUp]
					}
					//必经点 连通性已确认不是墙
					//nowNode = m.Nodes[focusIntXLeft+1][YUp-1] //中点右下
					if m.isWall_DynamicRayPathFind(focusIntXLeft+1, YUp-1) {
						return false, m.Nodes[focusIntXLeft+1][YUp-1] //中点右下
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					//nowNode = m.Nodes[intDynamicX][YUp] //上
					if m.isWall_DynamicRayPathFind(intDynamicX, YUp) {
						return false, m.Nodes[intDynamicX][YUp] //上
					}
					//nowNode = m.Nodes[intDynamicX][YUp-1] //下
					if m.isWall_DynamicRayPathFind(intDynamicX, YUp-1) {
						return false, m.Nodes[intDynamicX][YUp-1] //下
					}
				}
			}
		} else if isYKXBLeftDown {
			//函数图左下
			//     xy1 start
			//    /
			//xy2终点
			smallY = y2
			bigY = y1
			//通用遍历上下
			for YUp := bigY; YUp > smallY; YUp-- {
				//0.重置当前Y与交点上下判断
				focusButLeft = false
				focusButRight = false
				//1.根据不断增加的Y获得X
				dynamicX = (float64(YUp) - b) / k
				intDynamicX = int(dynamicX)
				//2.计算当前X与当前X的floor差值
				if math.Ceil(dynamicX) == math.Floor(dynamicX) {
					focusButLeft, focusButRight = true, true
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				} else if math.Ceil(dynamicX)-dynamicX <= epsilon { //focusDis必定大于0
					focusButLeft = true         //当前X值在交点左
					focusIntXLeft = intDynamicX //当前左X索引
				} else if dynamicX-math.Floor(dynamicX) <= epsilon { //focusDis必定大于0
					focusButRight = true            //当前X值在交点右
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				}
				//3.如果差值小于某个数或相等 认为碰到交点
				if focusButLeft || focusButRight {
					//4.判断必经点 擦边点 注意：不同方向必经点不一样
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][focusIntYUp-1], m.Nodes[XRight][focusIntYUp],
					//	m.Nodes[XRight-1][focusIntYUp], m.Nodes[XRight][focusIntYUp-1])
					//5.如果当前方向是 /
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					//nowNode = m.Nodes[focusIntXLeft+1][YUp] //中点右上
					if m.isWall_DynamicRayPathFind(focusIntXLeft+1, YUp) {
						return false, m.Nodes[focusIntXLeft+1][YUp] //中点右上
					}
					//擦边点 连通性未确认不是墙
					//nowNode1 = m.Nodes[focusIntXLeft][YUp]
					//nowNode2 = m.Nodes[focusIntXLeft+1][YUp-1]
					if m.isWall_DynamicRayPathFind(focusIntXLeft, YUp) && m.isWall_DynamicRayPathFind(focusIntXLeft+1, YUp-1) { //中点左上 //中点右下
						return false, m.Nodes[focusIntXLeft][YUp]
					}
					//必经点 连通性已确认不是墙
					//nowNode = m.Nodes[focusIntXLeft][YUp-1] //中点左下
					if m.isWall_DynamicRayPathFind(focusIntXLeft, YUp-1) {
						return false, m.Nodes[focusIntXLeft][YUp-1] //中点左下
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					//nowNode = m.Nodes[intDynamicX][YUp] //上
					if m.isWall_DynamicRayPathFind(intDynamicX, YUp) {
						return false, m.Nodes[intDynamicX][YUp] //上
					}
					//nowNode = m.Nodes[intDynamicX][YUp-1] //下
					if m.isWall_DynamicRayPathFind(intDynamicX, YUp-1) {
						return false, m.Nodes[intDynamicX][YUp-1] //下
					}
				}
			}
		} else if isYKXBLeftUp {
			//函数图左上
			//xy2
			//    \
			//    xy1 start
			smallY = y1
			bigY = y2
			//通用遍历上下
			for YUp := smallY + 1; YUp <= bigY; YUp++ {
				//0.重置当前Y与交点上下判断
				focusButLeft = false
				focusButRight = false
				//1.根据不断增加的Y获得X
				dynamicX = (float64(YUp) - b) / k
				intDynamicX = int(dynamicX)
				//2.计算当前X与当前X的floor差值
				if math.Ceil(dynamicX) == math.Floor(dynamicX) {
					focusButLeft, focusButRight = true, true
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				} else if math.Ceil(dynamicX)-dynamicX <= epsilon { //focusDis必定大于0
					focusButLeft = true         //当前X值在交点左
					focusIntXLeft = intDynamicX //当前左X索引
				} else if dynamicX-math.Floor(dynamicX) <= epsilon { //focusDis必定大于0
					focusButRight = true            //当前X值在交点右
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				}
				//3.如果差值小于某个数或相等 认为碰到交点
				if focusButLeft || focusButRight {
					//4.判断必经点 擦边点 注意：不同方向必经点不一样
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][focusIntYUp-1], m.Nodes[XRight][focusIntYUp],
					//	m.Nodes[XRight-1][focusIntYUp], m.Nodes[XRight][focusIntYUp-1])
					//5.如果当前方向是 /
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					//nowNode = m.Nodes[focusIntXLeft+1][YUp-1] //中点右下
					if m.isWall_DynamicRayPathFind(focusIntXLeft+1, YUp-1) {
						return false, m.Nodes[focusIntXLeft+1][YUp-1] //中点右下
					}
					//擦边点 连通性未确认不是墙
					//nowNode1 = m.Nodes[focusIntXLeft+1][YUp]
					//nowNode2 = m.Nodes[focusIntXLeft][YUp-1]
					if m.isWall_DynamicRayPathFind(focusIntXLeft+1, YUp) && m.isWall_DynamicRayPathFind(focusIntXLeft, YUp-1) { //中点右上 //中点左下
						return false, m.Nodes[focusIntXLeft+1][YUp]
					}
					//必经点 连通性已确认不是墙
					//nowNode = m.Nodes[focusIntXLeft][YUp] //中点左上
					if m.isWall_DynamicRayPathFind(focusIntXLeft, YUp) {
						return false, m.Nodes[focusIntXLeft][YUp] //中点左上
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					//nowNode = m.Nodes[intDynamicX][YUp-1] //下
					if m.isWall_DynamicRayPathFind(intDynamicX, YUp-1) {
						return false, m.Nodes[intDynamicX][YUp-1] //下
					}
					//nowNode = m.Nodes[intDynamicX][YUp] //上
					if m.isWall_DynamicRayPathFind(intDynamicX, YUp) {
						return false, m.Nodes[intDynamicX][YUp] //上
					}
				}
			}
		}
	}
	//最终y=kx+b没有false则判定相连
	return true, nil
}

// GetLineObstacleMaxSize 获取两点连线经过障碍物的最大尺寸
func (m *Map) GetLineObstacleMaxSize(x1, y1, x2, y2 int) int {
	//这一步 传入指针切片，内部把不是墙的节点添加完毕后传给外部
	var nowNode *Node
	var maxObstacleSize = 0
	tmpMaxSize := 0
	//judgeNodeSlice := make([]*Node, 0, 20)
	//1.直线同1行
	if x1 == x2 {
		if y1 < y2 {
			//1start  2
			for newY := y1 + 1; newY < y2; newY++ {
				//如果直线障碍点有1个是墙 直接返回false
				nowNode = m.Nodes[x1][newY]
				if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
					maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
				}
			}
		} else {
			//2 1start
			for newY := y1 - 1; newY > y2; newY-- {
				//如果直线障碍点有1个是墙 直接返回false
				nowNode = m.Nodes[x1][newY]
				if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
					maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
				}
			}
		}
		//遍历完返回
		return maxObstacleSize
	}
	//2.直线同1列
	if y1 == y2 {
		if x1 < x2 {
			//1 Small start
			//2 big
			for newX := x1 + 1; newX < x2; newX++ {
				//如果直线障碍点有1个是墙 直接返回false
				nowNode = m.Nodes[newX][y1]
				if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
					maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
				}
			}
		} else {
			//2 Small
			//1 big start
			for newX := x1 - 1; newX > x2; newX-- {
				//如果直线障碍点有1个是墙 直接返回false
				nowNode = m.Nodes[newX][y1]
				if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
					maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
				}
			}
		}
		//遍历完返回
		return maxObstacleSize
	}
	//0.先计算dx dy,用后面的点减去前面的点
	dx := x2 - x1
	dy := y2 - y1
	//3.处于标准的正方形对角线2点 判断是不是标准正方形 斜向45度 只选择必经点 只选择必经点 只选择必经点 只选择必经点!!!
	if dx == dy || dx == -dy {
		newX := x1
		newY := y1
		//1.打印右下斜
		//xy1 start
		//    \
		//      xy2终点
		if dx > 0 && dy > 0 {
			for {
				//1.右和下 擦边点2个都是墙 才不能相连
				if m.Nodes[newX][newY+1].IsWall() && m.Nodes[newX+1][newY].IsWall() {
					tmpMaxSize = getBiggerInt(m.obstacleNodeMaxSizeDic[m.Nodes[newX][newY+1]], m.obstacleNodeMaxSizeDic[m.Nodes[newX+1][newY]])
					if tmpMaxSize > maxObstacleSize {
						maxObstacleSize = tmpMaxSize
					}
				}
				//2.再改变当前点索引
				newX++
				newY++
				if newX == x2 && newY == y2 {
					return maxObstacleSize //到终点了
				}
				//必经点是墙
				nowNode = m.Nodes[newX][newY]
				if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
					maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
				}
			}
		}
		//2.打印左下斜
		//      xy1 start
		//    /
		//xy2终点
		if dx > 0 && dy < 0 {
			for {
				//1.左和下 擦边点2个都是墙
				if m.Nodes[newX][newY-1].IsWall() && m.Nodes[newX+1][newY].IsWall() {
					tmpMaxSize = getBiggerInt(m.obstacleNodeMaxSizeDic[m.Nodes[newX][newY-1]], m.obstacleNodeMaxSizeDic[m.Nodes[newX+1][newY]])
					if tmpMaxSize > maxObstacleSize {
						maxObstacleSize = tmpMaxSize
					}
				}
				//2.再改变当前点索引
				newX++
				newY--
				if newX == x2 && newY == y2 {
					return maxObstacleSize //到终点了
				}
				//必经点是墙
				nowNode = m.Nodes[newX][newY]
				if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
					maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
				}
			}
		}
		//3.打印左上斜
		//xy2
		//    \
		//      xy1 start
		if dx < 0 && dy < 0 {
			for {
				//1.上和左 擦边点2个都是墙
				if m.Nodes[newX-1][newY].IsWall() && m.Nodes[newX][newY-1].IsWall() {
					tmpMaxSize = getBiggerInt(m.obstacleNodeMaxSizeDic[m.Nodes[newX-1][newY]], m.obstacleNodeMaxSizeDic[m.Nodes[newX][newY-1]])
					if tmpMaxSize > maxObstacleSize {
						maxObstacleSize = tmpMaxSize
					}
				}
				//2.再改变当前点索引
				newX--
				newY--
				if newX == x2 && newY == y2 {
					return maxObstacleSize //到终点了
				}
				//必经点是墙
				nowNode = m.Nodes[newX][newY]
				if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
					maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
				}
			}
		}
		//4.打印右上斜
		//      xy2
		//    /
		//xy1
		if dx < 0 && dy > 0 {
			for {
				//1.上和右 擦边点2个都是墙
				if m.Nodes[newX-1][newY].IsWall() && m.Nodes[newX][newY+1].IsWall() {
					tmpMaxSize = getBiggerInt(m.obstacleNodeMaxSizeDic[m.Nodes[newX-1][newY]], m.obstacleNodeMaxSizeDic[m.Nodes[newX][newY+1]])
					if tmpMaxSize > maxObstacleSize {
						maxObstacleSize = tmpMaxSize
					}
				}
				//2.再改变当前点索引
				newX--
				newY++
				if newX == x2 && newY == y2 {
					return maxObstacleSize //到终点了
				}
				//已知必经点不是墙
				//必经点是墙
				nowNode = m.Nodes[newX][newY]
				if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
					maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
				}
			}
		}
	}
	//0.前面已经计算
	//dx := x2 - x1
	//dy := y2 - y1
	//4.y = kx +b 的情况
	//fmt.Println("处于y = kx +b的情况")
	//1.计算 k 和 b
	k, b := 0.0, 0.0
	k = float64(dy) / float64(dx)
	b = float64(y1) + 0.5 - (k * (float64(x1) + 0.5))
	isYKXBRightUp := false
	isYKXBRightDown := false
	isYKXBLeftDown := false
	isYKXBLeftUp := false
	//判断方向
	if dx > 0 && dy > 0 {
		//1.打印右下斜
		//xy1 start
		//    \
		//      xy2终点
		//函数图右上
		isYKXBRightUp = true
	} else if dx > 0 && dy < 0 {
		//2.打印左下斜
		//      xy1 start
		//    /
		//xy2终点
		//函数图右下
		isYKXBRightDown = true
	} else if dx < 0 && dy < 0 {
		//3.打印左上斜
		//xy2
		//    \
		//      xy1 start
		//函数图左下
		isYKXBLeftDown = true
	} else if dx < 0 && dy > 0 {
		//4.打印右上斜
		//      xy2
		//    /
		//xy1
		//函数图左上
		isYKXBLeftUp = true
	}
	//
	smallX, smallY, bigX, bigY := 0, 0, 0, 0
	//判断选择哪个轴
	if absInt(dx) > absInt(dy) {
		//选择x轴作为变量 动态Y判断左右
		focusIntYUp := 0
		focusButUp := false
		focusButDown := false
		dynamicY := 0.0
		intDynamicY := 0
		//判断函数图方向中,谁的索引差大就选择索引轴(X或Y轴)作为遍历去计算另一个值
		if isYKXBRightUp {
			//函数图右上
			//      xy2
			//    /
			//xy1 start
			smallX = x1
			bigX = x2
			//通用遍历左右
			for XRight := smallX + 1; XRight <= bigX; XRight++ {
				//0.重置当前Y与交点上下判断
				focusButUp = false
				focusButDown = false
				//1.根据不断增加的X获得Y
				dynamicY = (k * float64(XRight)) + b
				intDynamicY = int(dynamicY)
				//2.计算当前Y与当前Y的floor差值
				//3.特殊情况:刚好处于交点 也就是Floor和Ceil相等
				if math.Floor(dynamicY) == math.Ceil(dynamicY) {
					focusButUp, focusButDown = true, true
					focusIntYUp = intDynamicY //当前上Y索引 去掉小数
				} else if dynamicY-math.Floor(dynamicY) < 0.01 { //focusDis必定大于0
					focusButUp = true         //当前Y值在交点上方
					focusIntYUp = intDynamicY //当前上Y索引
				} else if math.Ceil(dynamicY)-dynamicY <= 0.01 { //focusDis必定大于0
					focusButDown = true           //当前Y值在交点下方
					focusIntYUp = intDynamicY + 1 //当前上Y索引
				}
				//3.如果差值小于某个数 认为碰到交点
				if focusButUp || focusButDown {
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[XRight-1][focusIntYUp-1] //中点左下
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
					//擦边点 连通性未确认不是墙
					if m.Nodes[XRight-1][focusIntYUp].IsWall() && m.Nodes[XRight][focusIntYUp-1].IsWall() { //中点左上 //中点右下
						tmpMaxSize = getBiggerInt(m.obstacleNodeMaxSizeDic[m.Nodes[XRight-1][focusIntYUp]], m.obstacleNodeMaxSizeDic[m.Nodes[XRight][focusIntYUp-1]])
						if tmpMaxSize > maxObstacleSize {
							maxObstacleSize = tmpMaxSize
						}
					}
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight][focusIntYUp] //中点右上
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
				} else {
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight-1][intDynamicY] //左
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
					nowNode = m.Nodes[XRight][intDynamicY] //右
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
				}
			}
		} else if isYKXBRightDown {
			//函数图右下
			//xy1 start
			//    \
			//      xy2
			smallX = x1
			bigX = x2
			//通用遍历左右
			for XRight := smallX + 1; XRight <= bigX; XRight++ {
				//0.重置当前Y与交点上下判断
				focusButUp = false
				focusButDown = false
				//1.根据不断增加的X获得Y
				dynamicY = (k * float64(XRight)) + b
				intDynamicY = int(dynamicY)
				//2.计算当前Y与当前Y的floor差值
				//3.特殊情况:刚好处于交点 也就是Floor和Ceil相等
				if math.Floor(dynamicY) == math.Ceil(dynamicY) {
					focusButUp, focusButDown = true, true
					focusIntYUp = intDynamicY //当前上Y索引 去掉小数
				} else if dynamicY-math.Floor(dynamicY) < 0.01 { //focusDis必定大于0
					focusButUp = true         //当前Y值在交点上方
					focusIntYUp = intDynamicY //当前上Y索引
				} else if math.Ceil(dynamicY)-dynamicY <= 0.01 { //focusDis必定大于0
					focusButDown = true           //当前Y值在交点下方
					focusIntYUp = intDynamicY + 1 //当前上Y索引
				}
				//3.如果差值小于某个数 认为碰到交点
				if focusButUp || focusButDown {
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[XRight-1][focusIntYUp] //中点左上
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
					//擦边点 连通性未确认不是墙
					if m.Nodes[XRight][focusIntYUp].IsWall() && m.Nodes[XRight-1][focusIntYUp-1].IsWall() { //中点右上 //中点左下
						tmpMaxSize = getBiggerInt(m.obstacleNodeMaxSizeDic[m.Nodes[XRight][focusIntYUp]], m.obstacleNodeMaxSizeDic[m.Nodes[XRight-1][focusIntYUp-1]])
						if tmpMaxSize > maxObstacleSize {
							maxObstacleSize = tmpMaxSize
						}
					}
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight][focusIntYUp-1] //中点右下
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight-1][intDynamicY] //左
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
					nowNode = m.Nodes[XRight][intDynamicY] //右
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
				}
			}
		} else if isYKXBLeftDown {
			//函数图左下
			//     xy1 start
			//    /
			//xy2
			smallX = x2
			bigX = x1
			//通用遍历左右
			for XRight := bigX; XRight > smallX; XRight-- {
				//0.重置当前Y与交点上下判断
				focusButUp = false
				focusButDown = false
				//1.根据不断增加的X获得Y
				dynamicY = (k * float64(XRight)) + b
				intDynamicY = int(dynamicY)
				//2.计算当前Y与当前Y的floor差值
				//3.特殊情况:刚好处于交点 也就是Floor和Ceil相等
				if math.Floor(dynamicY) == math.Ceil(dynamicY) {
					focusButUp, focusButDown = true, true
					focusIntYUp = intDynamicY //当前上Y索引 去掉小数
				} else if dynamicY-math.Floor(dynamicY) < 0.01 { //focusDis必定大于0
					focusButUp = true         //当前Y值在交点上方
					focusIntYUp = intDynamicY //当前上Y索引
				} else if math.Ceil(dynamicY)-dynamicY <= 0.01 { //focusDis必定大于0
					focusButDown = true           //当前Y值在交点下方
					focusIntYUp = intDynamicY + 1 //当前上Y索引
				}
				//3.如果差值小于某个数 认为碰到交点
				if focusButUp || focusButDown {
					//4.判断必经点 擦边点 注意：不同方向必经点不一样
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][focusIntYUp-1], m.Nodes[XRight][focusIntYUp],
					//	m.Nodes[XRight-1][focusIntYUp], m.Nodes[XRight][focusIntYUp-1])
					//5.如果当前方向是 /
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight][focusIntYUp] //中点右上
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
					//擦边点 连通性未确认不是墙
					if m.Nodes[XRight-1][focusIntYUp].IsWall() && m.Nodes[XRight][focusIntYUp-1].IsWall() { //中点左上 //中点右下
						tmpMaxSize = getBiggerInt(m.obstacleNodeMaxSizeDic[m.Nodes[XRight-1][focusIntYUp]], m.obstacleNodeMaxSizeDic[m.Nodes[XRight][focusIntYUp-1]])
						if tmpMaxSize > maxObstacleSize {
							maxObstacleSize = tmpMaxSize
						}
					}
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[XRight-1][focusIntYUp-1] //中点左下
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight][intDynamicY] //右
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
					nowNode = m.Nodes[XRight-1][intDynamicY] //左
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
				}
			}
		} else if isYKXBLeftUp {
			//函数图左上
			//xy2
			//    \
			//    xy1 start
			smallX = x2
			bigX = x1
			//通用遍历左右
			for XRight := bigX; XRight > smallX; XRight-- {
				//0.重置当前Y与交点上下判断
				focusButUp = false
				focusButDown = false
				//1.根据不断增加的X获得Y
				dynamicY = (k * float64(XRight)) + b
				intDynamicY = int(dynamicY)
				//2.计算当前Y与当前Y的floor差值
				//3.特殊情况:刚好处于交点 也就是Floor和Ceil相等
				if math.Floor(dynamicY) == math.Ceil(dynamicY) {
					focusButUp, focusButDown = true, true
					focusIntYUp = intDynamicY //当前上Y索引 去掉小数
				} else if dynamicY-math.Floor(dynamicY) < 0.01 { //focusDis必定大于0
					focusButUp = true         //当前Y值在交点上方
					focusIntYUp = intDynamicY //当前上Y索引
				} else if math.Ceil(dynamicY)-dynamicY <= 0.01 { //focusDis必定大于0
					focusButDown = true           //当前Y值在交点下方
					focusIntYUp = intDynamicY + 1 //当前上Y索引
				}
				//3.如果差值小于某个数 认为碰到交点
				if focusButUp || focusButDown {
					//4.判断必经点 擦边点 注意：不同方向必经点不一样
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][focusIntYUp-1], m.Nodes[XRight][focusIntYUp],
					//	m.Nodes[XRight-1][focusIntYUp], m.Nodes[XRight][focusIntYUp-1])
					//5.如果当前方向是 /
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight][focusIntYUp-1] //中点右下
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
					//擦边点 连通性未确认不是墙
					if m.Nodes[XRight-1][focusIntYUp-1].IsWall() && m.Nodes[XRight][focusIntYUp].IsWall() { //中点左下  //中点右上
						tmpMaxSize = getBiggerInt(m.obstacleNodeMaxSizeDic[m.Nodes[XRight-1][focusIntYUp-1]], m.obstacleNodeMaxSizeDic[m.Nodes[XRight][focusIntYUp]])
						if tmpMaxSize > maxObstacleSize {
							maxObstacleSize = tmpMaxSize
						}
					}
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[XRight-1][focusIntYUp] //中点左上
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight][intDynamicY] //右
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
					nowNode = m.Nodes[XRight-1][intDynamicY] //左
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
				}
			}
		}
	} else { //dx < dy
		//选择y轴作为变量 判断上下
		focusIntXLeft := 0
		focusButLeft := false
		focusButRight := false
		dynamicX := 0.0
		intDynamicX := 0
		//判断函数图方向中,谁的索引差大就选择索引轴(X或Y轴)作为遍历去计算另一个值
		if isYKXBRightUp {
			//函数图右上
			//      xy2
			//    /
			//xy1 start
			smallY = y1
			bigY = y2
			//通用遍历上下
			for YUp := smallY + 1; YUp <= bigY; YUp++ {
				//0.重置当前Y与交点上下判断
				focusButLeft = false
				focusButRight = false
				//1.根据不断增加的Y获得X
				dynamicX = (float64(YUp) - b) / k
				intDynamicX = int(dynamicX)
				//2.计算当前X与当前X的floor差值
				if math.Ceil(dynamicX) == math.Floor(dynamicX) {
					focusButLeft, focusButRight = true, true
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				} else if math.Ceil(dynamicX)-dynamicX < 0.01 { //focusDis必定大于0
					focusButLeft = true         //当前X值在交点左
					focusIntXLeft = intDynamicX //当前左X索引
				} else if dynamicX-math.Floor(dynamicX) < 0.01 { //focusDis必定大于0
					focusButRight = true            //当前X值在交点右
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				}
				//3.如果差值小于某个数或相等 认为碰到交点
				if focusButLeft || focusButRight {
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft][YUp-1] //中点左下
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
					//擦边点 连通性未确认不是墙
					if m.Nodes[focusIntXLeft+1][YUp-1].IsWall() && m.Nodes[focusIntXLeft][YUp].IsWall() { //中点右下 //中点左上
						tmpMaxSize = getBiggerInt(m.obstacleNodeMaxSizeDic[m.Nodes[focusIntXLeft+1][YUp-1]], m.obstacleNodeMaxSizeDic[m.Nodes[focusIntXLeft][YUp]])
						if tmpMaxSize > maxObstacleSize {
							maxObstacleSize = tmpMaxSize
						}
					}
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft+1][YUp] //中点右上
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[intDynamicX][YUp-1] //下
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
					nowNode = m.Nodes[intDynamicX][YUp] //上
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
				}
			}
		} else if isYKXBRightDown {
			//函数图右下
			//xy1 start
			//    \
			//      xy2
			smallY = y2
			bigY = y1
			//通用遍历上下
			for YUp := bigY; YUp > smallY; YUp-- {
				//0.重置当前Y与交点上下判断
				focusButLeft = false
				focusButRight = false
				//1.根据不断增加的Y获得X
				dynamicX = (float64(YUp) - b) / k
				intDynamicX = int(dynamicX)
				//2.计算当前X与当前X的floor差值
				if math.Ceil(dynamicX) == math.Floor(dynamicX) {
					focusButLeft, focusButRight = true, true
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				} else if math.Ceil(dynamicX)-dynamicX < 0.01 { //focusDis必定大于0
					focusButLeft = true         //当前X值在交点左
					focusIntXLeft = intDynamicX //当前左X索引
				} else if dynamicX-math.Floor(dynamicX) < 0.01 { //focusDis必定大于0
					focusButRight = true            //当前X值在交点右
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				}
				//3.如果差值小于某个数或相等 认为碰到交点
				if focusButLeft || focusButRight {
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft][YUp] //中点左上
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
					//擦边点 连通性未确认不是墙
					if m.Nodes[focusIntXLeft+1][YUp].IsWall() && m.Nodes[focusIntXLeft][YUp-1].IsWall() { //中点右上  //中点左下
						tmpMaxSize = getBiggerInt(m.obstacleNodeMaxSizeDic[m.Nodes[focusIntXLeft+1][YUp]], m.obstacleNodeMaxSizeDic[m.Nodes[focusIntXLeft][YUp-1]])
						if tmpMaxSize > maxObstacleSize {
							maxObstacleSize = tmpMaxSize
						}
					}
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft+1][YUp-1] //中点右下
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[intDynamicX][YUp] //上
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
					nowNode = m.Nodes[intDynamicX][YUp-1] //下
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
				}
			}
		} else if isYKXBLeftDown {
			//函数图左下
			//     xy1 start
			//    /
			//xy2终点
			smallY = y2
			bigY = y1
			//通用遍历上下
			for YUp := bigY; YUp > smallY; YUp-- {
				//0.重置当前Y与交点上下判断
				focusButLeft = false
				focusButRight = false
				//1.根据不断增加的Y获得X
				dynamicX = (float64(YUp) - b) / k
				intDynamicX = int(dynamicX)
				//2.计算当前X与当前X的floor差值
				if math.Ceil(dynamicX) == math.Floor(dynamicX) {
					focusButLeft, focusButRight = true, true
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				} else if math.Ceil(dynamicX)-dynamicX < 0.01 { //focusDis必定大于0
					focusButLeft = true         //当前X值在交点左
					focusIntXLeft = intDynamicX //当前左X索引
				} else if dynamicX-math.Floor(dynamicX) < 0.01 { //focusDis必定大于0
					focusButRight = true            //当前X值在交点右
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				}
				//3.如果差值小于某个数或相等 认为碰到交点
				if focusButLeft || focusButRight {
					//4.判断必经点 擦边点 注意：不同方向必经点不一样
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][focusIntYUp-1], m.Nodes[XRight][focusIntYUp],
					//	m.Nodes[XRight-1][focusIntYUp], m.Nodes[XRight][focusIntYUp-1])
					//5.如果当前方向是 /
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft+1][YUp] //中点右上
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
					//擦边点 连通性未确认不是墙
					if m.Nodes[focusIntXLeft][YUp].IsWall() && m.Nodes[focusIntXLeft+1][YUp-1].IsWall() { //中点左上 //中点右下
						tmpMaxSize = getBiggerInt(m.obstacleNodeMaxSizeDic[m.Nodes[focusIntXLeft][YUp]], m.obstacleNodeMaxSizeDic[m.Nodes[focusIntXLeft+1][YUp-1]])
						if tmpMaxSize > maxObstacleSize {
							maxObstacleSize = tmpMaxSize
						}
					}
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft][YUp-1] //中点左下
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[intDynamicX][YUp] //上
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
					nowNode = m.Nodes[intDynamicX][YUp-1] //下
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
				}
			}
		} else if isYKXBLeftUp {
			//函数图左上
			//xy2
			//    \
			//    xy1 start
			smallY = y1
			bigY = y2
			//通用遍历上下
			for YUp := smallY + 1; YUp <= bigY; YUp++ {
				//0.重置当前Y与交点上下判断
				focusButLeft = false
				focusButRight = false
				//1.根据不断增加的Y获得X
				dynamicX = (float64(YUp) - b) / k
				intDynamicX = int(dynamicX)
				//2.计算当前X与当前X的floor差值
				if math.Ceil(dynamicX) == math.Floor(dynamicX) {
					focusButLeft, focusButRight = true, true
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				} else if math.Ceil(dynamicX)-dynamicX < 0.01 { //focusDis必定大于0
					focusButLeft = true         //当前X值在交点左
					focusIntXLeft = intDynamicX //当前左X索引
				} else if dynamicX-math.Floor(dynamicX) < 0.01 { //focusDis必定大于0
					focusButRight = true            //当前X值在交点右
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				}
				//3.如果差值小于某个数或相等 认为碰到交点
				if focusButLeft || focusButRight {
					//4.判断必经点 擦边点 注意：不同方向必经点不一样
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][focusIntYUp-1], m.Nodes[XRight][focusIntYUp],
					//	m.Nodes[XRight-1][focusIntYUp], m.Nodes[XRight][focusIntYUp-1])
					//5.如果当前方向是 /
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft+1][YUp-1] //中点右下
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
					//擦边点 连通性未确认不是墙
					if m.Nodes[focusIntXLeft+1][YUp].IsWall() && m.Nodes[focusIntXLeft][YUp-1].IsWall() { //中点右上 //中点左下
						tmpMaxSize = getBiggerInt(m.obstacleNodeMaxSizeDic[m.Nodes[focusIntXLeft+1][YUp]], m.obstacleNodeMaxSizeDic[m.Nodes[focusIntXLeft][YUp-1]])
						if tmpMaxSize > maxObstacleSize {
							maxObstacleSize = tmpMaxSize
						}
					}
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft][YUp] //中点左上
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[intDynamicX][YUp-1] //下
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
					nowNode = m.Nodes[intDynamicX][YUp] //上
					if nowNode.IsWall() && m.obstacleNodeMaxSizeDic[nowNode] > maxObstacleSize {
						maxObstacleSize = m.obstacleNodeMaxSizeDic[nowNode]
					}
				}
			}
		}
	}
	//最终y=kx+b没有false则判定相连
	return maxObstacleSize
}

// GetLineObstacleMaxSizeImpSpeed 加速获取2点任一最近的障碍尺寸
func (m *Map) GetLineObstacleMaxSizeImpSpeed(x1, y1, x2, y2 int) int {
	//这一步 传入指针切片，内部把不是墙的节点添加完毕后传给外部
	var nowNode *Node
	var maxObstacleSize = 0
	//judgeNodeSlice := make([]*Node, 0, 20)
	//1.直线同1行
	if x1 == x2 {
		if y1 < y2 {
			//1start  2
			for newY := y1 + 1; newY < y2; newY++ {
				//如果直线障碍点有1个是墙 直接返回false
				nowNode = m.Nodes[x1][newY]
				if nowNode.IsWall() {
					return m.obstacleNodeMaxSizeDic[nowNode]
				}
			}
		} else {
			//2 1start
			for newY := y1 - 1; newY > y2; newY-- {
				//如果直线障碍点有1个是墙 直接返回false
				nowNode = m.Nodes[x1][newY]
				if nowNode.IsWall() {
					return m.obstacleNodeMaxSizeDic[nowNode]
				}
			}
		}
	}
	//2.直线同1列
	if y1 == y2 {
		if x1 < x2 {
			//1 Small start
			//2 big
			for newX := x1 + 1; newX < x2; newX++ {
				//如果直线障碍点有1个是墙 直接返回false
				nowNode = m.Nodes[newX][y1]
				if nowNode.IsWall() {
					return m.obstacleNodeMaxSizeDic[nowNode]
				}
			}
		} else {
			//2 Small
			//1 big start
			for newX := x1 - 1; newX > x2; newX-- {
				//如果直线障碍点有1个是墙 直接返回false
				nowNode = m.Nodes[newX][y1]
				if nowNode.IsWall() {
					return m.obstacleNodeMaxSizeDic[nowNode]
				}
			}
		}
	}
	//0.先计算dx dy,用后面的点减去前面的点
	dx := x2 - x1
	dy := y2 - y1
	//3.处于标准的正方形对角线2点 判断是不是标准正方形 斜向45度 只选择必经点 只选择必经点 只选择必经点 只选择必经点!!!
	if dx == dy || dx == -dy {
		newX := x1
		newY := y1
		//1.打印右下斜
		//xy1 start
		//    \
		//      xy2终点
		if dx > 0 && dy > 0 {
			for {
				//1.右和下 擦边点2个都是墙 才不能相连
				if m.Nodes[newX][newY+1].IsWall() && m.Nodes[newX+1][newY].IsWall() {
					return getBiggerInt(m.obstacleNodeMaxSizeDic[m.Nodes[newX][newY+1]], m.obstacleNodeMaxSizeDic[m.Nodes[newX+1][newY]])
				}
				//2.再改变当前点索引
				newX++
				newY++
				if newX == x2 && newY == y2 {
					return maxObstacleSize //到终点了
				}
				//必经点是墙
				nowNode = m.Nodes[newX][newY]
				if nowNode.IsWall() {
					return m.obstacleNodeMaxSizeDic[nowNode]
				}
			}
		}
		//2.打印左下斜
		//      xy1 start
		//    /
		//xy2终点
		if dx > 0 && dy < 0 {
			for {
				//1.左和下 擦边点2个都是墙
				if m.Nodes[newX][newY-1].IsWall() && m.Nodes[newX+1][newY].IsWall() {
					return getBiggerInt(m.obstacleNodeMaxSizeDic[m.Nodes[newX][newY-1]], m.obstacleNodeMaxSizeDic[m.Nodes[newX+1][newY]])
				}
				//2.再改变当前点索引
				newX++
				newY--
				if newX == x2 && newY == y2 {
					return maxObstacleSize //到终点了
				}
				//必经点是墙
				nowNode = m.Nodes[newX][newY]
				if nowNode.IsWall() {
					return m.obstacleNodeMaxSizeDic[nowNode]
				}
			}
		}
		//3.打印左上斜
		//xy2
		//    \
		//      xy1 start
		if dx < 0 && dy < 0 {
			for {
				//1.上和左 擦边点2个都是墙
				if m.Nodes[newX-1][newY].IsWall() && m.Nodes[newX][newY-1].IsWall() {
					return getBiggerInt(m.obstacleNodeMaxSizeDic[m.Nodes[newX-1][newY]], m.obstacleNodeMaxSizeDic[m.Nodes[newX][newY-1]])
				}
				//2.再改变当前点索引
				newX--
				newY--
				if newX == x2 && newY == y2 {
					return maxObstacleSize //到终点了
				}
				//必经点是墙
				nowNode = m.Nodes[newX][newY]
				if nowNode.IsWall() {
					return m.obstacleNodeMaxSizeDic[nowNode]
				}
			}
		}
		//4.打印右上斜
		//      xy2
		//    /
		//xy1
		if dx < 0 && dy > 0 {
			for {
				//1.上和右 擦边点2个都是墙
				if m.Nodes[newX-1][newY].IsWall() && m.Nodes[newX][newY+1].IsWall() {
					return getBiggerInt(m.obstacleNodeMaxSizeDic[m.Nodes[newX-1][newY]], m.obstacleNodeMaxSizeDic[m.Nodes[newX][newY+1]])
				}
				//2.再改变当前点索引
				newX--
				newY++
				if newX == x2 && newY == y2 {
					return maxObstacleSize //到终点了
				}
				//已知必经点不是墙
				//必经点是墙
				nowNode = m.Nodes[newX][newY]
				if nowNode.IsWall() {
					return m.obstacleNodeMaxSizeDic[nowNode]
				}
			}
		}
	}
	//0.前面已经计算
	//dx := x2 - x1
	//dy := y2 - y1
	//4.y = kx +b 的情况
	//fmt.Println("处于y = kx +b的情况")
	//1.计算 k 和 b
	k, b := 0.0, 0.0
	k = float64(dy) / float64(dx)
	b = float64(y1) + 0.5 - (k * (float64(x1) + 0.5))
	isYKXBRightUp := false
	isYKXBRightDown := false
	isYKXBLeftDown := false
	isYKXBLeftUp := false
	//判断方向
	if dx > 0 && dy > 0 {
		//1.打印右下斜
		//xy1 start
		//    \
		//      xy2终点
		//函数图右上
		isYKXBRightUp = true
	} else if dx > 0 && dy < 0 {
		//2.打印左下斜
		//      xy1 start
		//    /
		//xy2终点
		//函数图右下
		isYKXBRightDown = true
	} else if dx < 0 && dy < 0 {
		//3.打印左上斜
		//xy2
		//    \
		//      xy1 start
		//函数图左下
		isYKXBLeftDown = true
	} else if dx < 0 && dy > 0 {
		//4.打印右上斜
		//      xy2
		//    /
		//xy1
		//函数图左上
		isYKXBLeftUp = true
	}
	//
	smallX, smallY, bigX, bigY := 0, 0, 0, 0
	//判断选择哪个轴
	if absInt(dx) > absInt(dy) {
		//选择x轴作为变量 动态Y判断左右
		focusIntYUp := 0
		focusButUp := false
		focusButDown := false
		dynamicY := 0.0
		intDynamicY := 0
		//判断函数图方向中,谁的索引差大就选择索引轴(X或Y轴)作为遍历去计算另一个值
		if isYKXBRightUp {
			//函数图右上
			//      xy2
			//    /
			//xy1 start
			smallX = x1
			bigX = x2
			//通用遍历左右
			for XRight := smallX + 1; XRight <= bigX; XRight++ {
				//0.重置当前Y与交点上下判断
				focusButUp = false
				focusButDown = false
				//1.根据不断增加的X获得Y
				dynamicY = (k * float64(XRight)) + b
				intDynamicY = int(dynamicY)
				//2.计算当前Y与当前Y的floor差值
				//3.特殊情况:刚好处于交点 也就是Floor和Ceil相等
				if math.Floor(dynamicY) == math.Ceil(dynamicY) {
					focusButUp, focusButDown = true, true
					focusIntYUp = intDynamicY //当前上Y索引 去掉小数
				} else if dynamicY-math.Floor(dynamicY) < 0.01 { //focusDis必定大于0
					focusButUp = true         //当前Y值在交点上方
					focusIntYUp = intDynamicY //当前上Y索引
				} else if math.Ceil(dynamicY)-dynamicY <= 0.01 { //focusDis必定大于0
					focusButDown = true           //当前Y值在交点下方
					focusIntYUp = intDynamicY + 1 //当前上Y索引
				}
				//3.如果差值小于某个数 认为碰到交点
				if focusButUp || focusButDown {
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[XRight-1][focusIntYUp-1] //中点左下
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
					//擦边点 连通性未确认不是墙
					if m.Nodes[XRight-1][focusIntYUp].IsWall() && m.Nodes[XRight][focusIntYUp-1].IsWall() { //中点左上 //中点右下
						return getBiggerInt(m.obstacleNodeMaxSizeDic[m.Nodes[XRight-1][focusIntYUp]], m.obstacleNodeMaxSizeDic[m.Nodes[XRight][focusIntYUp-1]])
					}
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight][focusIntYUp] //中点右上
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
				} else {
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight-1][intDynamicY] //左
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
					nowNode = m.Nodes[XRight][intDynamicY] //右
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
				}
			}
		} else if isYKXBRightDown {
			//函数图右下
			//xy1 start
			//    \
			//      xy2
			smallX = x1
			bigX = x2
			//通用遍历左右
			for XRight := smallX + 1; XRight <= bigX; XRight++ {
				//0.重置当前Y与交点上下判断
				focusButUp = false
				focusButDown = false
				//1.根据不断增加的X获得Y
				dynamicY = (k * float64(XRight)) + b
				intDynamicY = int(dynamicY)
				//2.计算当前Y与当前Y的floor差值
				//3.特殊情况:刚好处于交点 也就是Floor和Ceil相等
				if math.Floor(dynamicY) == math.Ceil(dynamicY) {
					focusButUp, focusButDown = true, true
					focusIntYUp = intDynamicY //当前上Y索引 去掉小数
				} else if dynamicY-math.Floor(dynamicY) < 0.01 { //focusDis必定大于0
					focusButUp = true         //当前Y值在交点上方
					focusIntYUp = intDynamicY //当前上Y索引
				} else if math.Ceil(dynamicY)-dynamicY <= 0.01 { //focusDis必定大于0
					focusButDown = true           //当前Y值在交点下方
					focusIntYUp = intDynamicY + 1 //当前上Y索引
				}
				//3.如果差值小于某个数 认为碰到交点
				if focusButUp || focusButDown {
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[XRight-1][focusIntYUp] //中点左上
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
					//擦边点 连通性未确认不是墙
					if m.Nodes[XRight][focusIntYUp].IsWall() && m.Nodes[XRight-1][focusIntYUp-1].IsWall() { //中点右上 //中点左下
						return getBiggerInt(m.obstacleNodeMaxSizeDic[m.Nodes[XRight][focusIntYUp]], m.obstacleNodeMaxSizeDic[m.Nodes[XRight-1][focusIntYUp-1]])
					}
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight][focusIntYUp-1] //中点右下
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight-1][intDynamicY] //左
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
					nowNode = m.Nodes[XRight][intDynamicY] //右
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
				}
			}
		} else if isYKXBLeftDown {
			//函数图左下
			//     xy1 start
			//    /
			//xy2
			smallX = x2
			bigX = x1
			//通用遍历左右
			for XRight := bigX; XRight > smallX; XRight-- {
				//0.重置当前Y与交点上下判断
				focusButUp = false
				focusButDown = false
				//1.根据不断增加的X获得Y
				dynamicY = (k * float64(XRight)) + b
				intDynamicY = int(dynamicY)
				//2.计算当前Y与当前Y的floor差值
				//3.特殊情况:刚好处于交点 也就是Floor和Ceil相等
				if math.Floor(dynamicY) == math.Ceil(dynamicY) {
					focusButUp, focusButDown = true, true
					focusIntYUp = intDynamicY //当前上Y索引 去掉小数
				} else if dynamicY-math.Floor(dynamicY) < 0.01 { //focusDis必定大于0
					focusButUp = true         //当前Y值在交点上方
					focusIntYUp = intDynamicY //当前上Y索引
				} else if math.Ceil(dynamicY)-dynamicY <= 0.01 { //focusDis必定大于0
					focusButDown = true           //当前Y值在交点下方
					focusIntYUp = intDynamicY + 1 //当前上Y索引
				}
				//3.如果差值小于某个数 认为碰到交点
				if focusButUp || focusButDown {
					//4.判断必经点 擦边点 注意：不同方向必经点不一样
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][focusIntYUp-1], m.Nodes[XRight][focusIntYUp],
					//	m.Nodes[XRight-1][focusIntYUp], m.Nodes[XRight][focusIntYUp-1])
					//5.如果当前方向是 /
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight][focusIntYUp] //中点右上
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
					//擦边点 连通性未确认不是墙
					if m.Nodes[XRight-1][focusIntYUp].IsWall() && m.Nodes[XRight][focusIntYUp-1].IsWall() { //中点左上 //中点右下
						return getBiggerInt(m.obstacleNodeMaxSizeDic[m.Nodes[XRight-1][focusIntYUp]], m.obstacleNodeMaxSizeDic[m.Nodes[XRight][focusIntYUp-1]])
					}
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[XRight-1][focusIntYUp-1] //中点左下
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight][intDynamicY] //右
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
					nowNode = m.Nodes[XRight-1][intDynamicY] //左
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
				}
			}
		} else if isYKXBLeftUp {
			//函数图左上
			//xy2
			//    \
			//    xy1 start
			smallX = x2
			bigX = x1
			//通用遍历左右
			for XRight := bigX; XRight > smallX; XRight-- {
				//0.重置当前Y与交点上下判断
				focusButUp = false
				focusButDown = false
				//1.根据不断增加的X获得Y
				dynamicY = (k * float64(XRight)) + b
				intDynamicY = int(dynamicY)
				//2.计算当前Y与当前Y的floor差值
				//3.特殊情况:刚好处于交点 也就是Floor和Ceil相等
				if math.Floor(dynamicY) == math.Ceil(dynamicY) {
					focusButUp, focusButDown = true, true
					focusIntYUp = intDynamicY //当前上Y索引 去掉小数
				} else if dynamicY-math.Floor(dynamicY) < 0.01 { //focusDis必定大于0
					focusButUp = true         //当前Y值在交点上方
					focusIntYUp = intDynamicY //当前上Y索引
				} else if math.Ceil(dynamicY)-dynamicY <= 0.01 { //focusDis必定大于0
					focusButDown = true           //当前Y值在交点下方
					focusIntYUp = intDynamicY + 1 //当前上Y索引
				}
				//3.如果差值小于某个数 认为碰到交点
				if focusButUp || focusButDown {
					//4.判断必经点 擦边点 注意：不同方向必经点不一样
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][focusIntYUp-1], m.Nodes[XRight][focusIntYUp],
					//	m.Nodes[XRight-1][focusIntYUp], m.Nodes[XRight][focusIntYUp-1])
					//5.如果当前方向是 /
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight][focusIntYUp-1] //中点右下
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
					//擦边点 连通性未确认不是墙
					if m.Nodes[XRight-1][focusIntYUp-1].IsWall() && m.Nodes[XRight][focusIntYUp].IsWall() { //中点左下  //中点右上
						return getBiggerInt(m.obstacleNodeMaxSizeDic[m.Nodes[XRight-1][focusIntYUp-1]], m.obstacleNodeMaxSizeDic[m.Nodes[XRight][focusIntYUp]])
					}
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[XRight-1][focusIntYUp] //中点左上
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight][intDynamicY] //右
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
					nowNode = m.Nodes[XRight-1][intDynamicY] //左
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
				}
			}
		}
	} else { //dx < dy
		//选择y轴作为变量 判断上下
		focusIntXLeft := 0
		focusButLeft := false
		focusButRight := false
		dynamicX := 0.0
		intDynamicX := 0
		//判断函数图方向中,谁的索引差大就选择索引轴(X或Y轴)作为遍历去计算另一个值
		if isYKXBRightUp {
			//函数图右上
			//      xy2
			//    /
			//xy1 start
			smallY = y1
			bigY = y2
			//通用遍历上下
			for YUp := smallY + 1; YUp <= bigY; YUp++ {
				//0.重置当前Y与交点上下判断
				focusButLeft = false
				focusButRight = false
				//1.根据不断增加的Y获得X
				dynamicX = (float64(YUp) - b) / k
				intDynamicX = int(dynamicX)
				//2.计算当前X与当前X的floor差值
				if math.Ceil(dynamicX) == math.Floor(dynamicX) {
					focusButLeft, focusButRight = true, true
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				} else if math.Ceil(dynamicX)-dynamicX < 0.01 { //focusDis必定大于0
					focusButLeft = true         //当前X值在交点左
					focusIntXLeft = intDynamicX //当前左X索引
				} else if dynamicX-math.Floor(dynamicX) < 0.01 { //focusDis必定大于0
					focusButRight = true            //当前X值在交点右
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				}
				//3.如果差值小于某个数或相等 认为碰到交点
				if focusButLeft || focusButRight {
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft][YUp-1] //中点左下
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
					//擦边点 连通性未确认不是墙
					if m.Nodes[focusIntXLeft+1][YUp-1].IsWall() && m.Nodes[focusIntXLeft][YUp].IsWall() { //中点右下 //中点左上
						return getBiggerInt(m.obstacleNodeMaxSizeDic[m.Nodes[focusIntXLeft+1][YUp-1]], m.obstacleNodeMaxSizeDic[m.Nodes[focusIntXLeft][YUp]])
					}
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft+1][YUp] //中点右上
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[intDynamicX][YUp-1] //下
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
					nowNode = m.Nodes[intDynamicX][YUp] //上
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
				}
			}
		} else if isYKXBRightDown {
			//函数图右下
			//xy1 start
			//    \
			//      xy2
			smallY = y2
			bigY = y1
			//通用遍历上下
			for YUp := bigY; YUp > smallY; YUp-- {
				//0.重置当前Y与交点上下判断
				focusButLeft = false
				focusButRight = false
				//1.根据不断增加的Y获得X
				dynamicX = (float64(YUp) - b) / k
				intDynamicX = int(dynamicX)
				//2.计算当前X与当前X的floor差值
				if math.Ceil(dynamicX) == math.Floor(dynamicX) {
					focusButLeft, focusButRight = true, true
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				} else if math.Ceil(dynamicX)-dynamicX < 0.01 { //focusDis必定大于0
					focusButLeft = true         //当前X值在交点左
					focusIntXLeft = intDynamicX //当前左X索引
				} else if dynamicX-math.Floor(dynamicX) < 0.01 { //focusDis必定大于0
					focusButRight = true            //当前X值在交点右
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				}
				//3.如果差值小于某个数或相等 认为碰到交点
				if focusButLeft || focusButRight {
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft][YUp] //中点左上
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
					//擦边点 连通性未确认不是墙
					if m.Nodes[focusIntXLeft+1][YUp].IsWall() && m.Nodes[focusIntXLeft][YUp-1].IsWall() { //中点右上  //中点左下
						return getBiggerInt(m.obstacleNodeMaxSizeDic[m.Nodes[focusIntXLeft+1][YUp]], m.obstacleNodeMaxSizeDic[m.Nodes[focusIntXLeft][YUp-1]])
					}
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft+1][YUp-1] //中点右下
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[intDynamicX][YUp] //上
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
					nowNode = m.Nodes[intDynamicX][YUp-1] //下
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
				}
			}
		} else if isYKXBLeftDown {
			//函数图左下
			//     xy1 start
			//    /
			//xy2终点
			smallY = y2
			bigY = y1
			//通用遍历上下
			for YUp := bigY; YUp > smallY; YUp-- {
				//0.重置当前Y与交点上下判断
				focusButLeft = false
				focusButRight = false
				//1.根据不断增加的Y获得X
				dynamicX = (float64(YUp) - b) / k
				intDynamicX = int(dynamicX)
				//2.计算当前X与当前X的floor差值
				if math.Ceil(dynamicX) == math.Floor(dynamicX) {
					focusButLeft, focusButRight = true, true
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				} else if math.Ceil(dynamicX)-dynamicX < 0.01 { //focusDis必定大于0
					focusButLeft = true         //当前X值在交点左
					focusIntXLeft = intDynamicX //当前左X索引
				} else if dynamicX-math.Floor(dynamicX) < 0.01 { //focusDis必定大于0
					focusButRight = true            //当前X值在交点右
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				}
				//3.如果差值小于某个数或相等 认为碰到交点
				if focusButLeft || focusButRight {
					//4.判断必经点 擦边点 注意：不同方向必经点不一样
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][focusIntYUp-1], m.Nodes[XRight][focusIntYUp],
					//	m.Nodes[XRight-1][focusIntYUp], m.Nodes[XRight][focusIntYUp-1])
					//5.如果当前方向是 /
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft+1][YUp] //中点右上
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
					//擦边点 连通性未确认不是墙
					if m.Nodes[focusIntXLeft][YUp].IsWall() && m.Nodes[focusIntXLeft+1][YUp-1].IsWall() { //中点左上 //中点右下
						return getBiggerInt(m.obstacleNodeMaxSizeDic[m.Nodes[focusIntXLeft][YUp]], m.obstacleNodeMaxSizeDic[m.Nodes[focusIntXLeft+1][YUp-1]])
					}
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft][YUp-1] //中点左下
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[intDynamicX][YUp] //上
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
					nowNode = m.Nodes[intDynamicX][YUp-1] //下
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
				}
			}
		} else if isYKXBLeftUp {
			//函数图左上
			//xy2
			//    \
			//    xy1 start
			smallY = y1
			bigY = y2
			//通用遍历上下
			for YUp := smallY + 1; YUp <= bigY; YUp++ {
				//0.重置当前Y与交点上下判断
				focusButLeft = false
				focusButRight = false
				//1.根据不断增加的Y获得X
				dynamicX = (float64(YUp) - b) / k
				intDynamicX = int(dynamicX)
				//2.计算当前X与当前X的floor差值
				if math.Ceil(dynamicX) == math.Floor(dynamicX) {
					focusButLeft, focusButRight = true, true
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				} else if math.Ceil(dynamicX)-dynamicX < 0.01 { //focusDis必定大于0
					focusButLeft = true         //当前X值在交点左
					focusIntXLeft = intDynamicX //当前左X索引
				} else if dynamicX-math.Floor(dynamicX) < 0.01 { //focusDis必定大于0
					focusButRight = true            //当前X值在交点右
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				}
				//3.如果差值小于某个数或相等 认为碰到交点
				if focusButLeft || focusButRight {
					//4.判断必经点 擦边点 注意：不同方向必经点不一样
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][focusIntYUp-1], m.Nodes[XRight][focusIntYUp],
					//	m.Nodes[XRight-1][focusIntYUp], m.Nodes[XRight][focusIntYUp-1])
					//5.如果当前方向是 /
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft+1][YUp-1] //中点右下
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
					//擦边点 连通性未确认不是墙
					if m.Nodes[focusIntXLeft+1][YUp].IsWall() && m.Nodes[focusIntXLeft][YUp-1].IsWall() { //中点右上 //中点左下
						return getBiggerInt(m.obstacleNodeMaxSizeDic[m.Nodes[focusIntXLeft+1][YUp]], m.obstacleNodeMaxSizeDic[m.Nodes[focusIntXLeft][YUp-1]])
					}
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft][YUp] //中点左上
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[intDynamicX][YUp-1] //下
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
					nowNode = m.Nodes[intDynamicX][YUp] //上
					if nowNode.IsWall() {
						return m.obstacleNodeMaxSizeDic[nowNode]
					}
				}
			}
		}
	}
	//最终y=kx+b没有false则判定相连
	return maxObstacleSize
}

// JudgeLineObstacleTestNew
//
//	func (m *Map) JudgeLineObstacleTestNew(x1, y1, x2, y2 int, midNotWallNodeSliceFromX1 *[]*Node) {
//		//这一步 传入指针切片，内部把不是墙的节点添加完毕后传给外部
//		var nowNode *Node
//		//judgeNodeSlice := make([]*Node, 0, 20)
//		//1.直线同1行
//		if x1 == x2 {
//			if y1 < y2 {
//				//1start  2
//				for newY := y1 + 1; newY < y2; newY++ {
//					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[x1][newY])
//					//如果直线障碍点有1个是墙 直接返回false
//					//if m.Nodes[x1][newY].IsWall()  {//如果中间点不是墙 本来就没有起点终点 不用判断当前是起点终点
//					//	return false
//					//}
//					nowNode = m.Nodes[x1][newY]
//					if !nowNode.IsWall() {
//						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode) //这个方法默认之前判断可以相连 默认不是墙 本来就没有起点终点 不用判断当前是起点终点
//					}
//				}
//			} else {
//				//2 1start
//				for newY := y1 - 1; newY > y2; newY-- {
//					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[x1][newY])
//					//如果直线障碍点有1个是墙 直接返回false
//					//if m.Nodes[x1][newY].IsWall()  {//如果中间点不是墙 本来就没有起点终点 不用判断当前是起点终点
//					//	return false
//					//}
//					nowNode = m.Nodes[x1][newY]
//					if !nowNode.IsWall() {
//						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode) //这个方法默认之前判断可以相连 默认不是墙 本来就没有起点终点 不用判断当前是起点终点
//					}
//				}
//			}
//			//记得退出
//			return
//		}
//		//2.直线同1列
//		if y1 == y2 {
//			if x1 < x2 {
//				//1 Small start
//				//2 big
//				for newX := x1 + 1; newX < x2; newX++ {
//					//如果直线障碍点有1个是墙 直接返回false
//					//if m.Nodes[newX][y1].IsWall() {
//					//	return false
//					//}
//					nowNode = m.Nodes[newX][y1]
//					if !nowNode.IsWall() {
//						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode) //这个方法默认之前判断可以相连 默认不是墙 本来就没有起点终点 不用判断当前是起点终点
//					}
//				}
//			} else {
//				//2 Small
//				//1 big start
//				for newX := x1 - 1; newX > x2; newX-- {
//					//如果直线障碍点有1个是墙 直接返回false
//					//if m.Nodes[newX][y1].IsWall() {
//					//	return false
//					//}
//					nowNode = m.Nodes[newX][y1]
//					if !nowNode.IsWall() {
//						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode) //这个方法默认之前判断可以相连 默认不是墙 本来就没有起点终点 不用判断当前是起点终点
//					}
//				}
//			}
//			//记得退出
//			return
//		}
//		//0.先计算dx dy,用后面的点减去前面的点
//		dx := x2 - x1
//		dy := y2 - y1
//		//3.处于标准的正方形对角线2点 判断是不是标准正方形 斜向45度 只选择必经点 只选择必经点 只选择必经点 只选择必经点!!!
//		if dx == dy || dx == -dy {
//			//1.打印右下斜
//			//xy1 start
//			//    \
//			//      xy2终点
//			if dx > 0 && dy > 0 {
//				newX := x1
//				newY := y1
//				for {
//					//1.先判断当前点的 右和下 2个都是墙
//					//if !m.Nodes[newX][newY+1].IsWall() && !m.Nodes[newX+1][newY].IsWall() {
//					//	*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, m.Nodes[newX][newY+1])
//					//	*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, m.Nodes[newX+1][newY])
//					//}
//					*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, m.Nodes[newX][newY+1])
//					*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, m.Nodes[newX+1][newY])
//					//2.再改变当前点索引
//					newX++
//					newY++
//					if newX == x2 && newY == y2 {
//						return
//					}
//					//已知必经点不是墙
//					nowNode = m.Nodes[newX][newY]
//					if !nowNode.IsWall() {
//						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//					}
//					//擦边点判断是否是墙 一定在界内
//				}
//			}
//			//2.打印左下斜
//			//      xy1 start
//			//    /
//			//xy2终点
//			if dx > 0 && dy < 0 {
//				newX := x1
//				newY := y1
//				for {
//					//1.先判断当前点的 左和下 2个都是墙
//					//if !m.Nodes[newX][newY-1].IsWall() && !m.Nodes[newX+1][newY].IsWall() {
//					//	*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, m.Nodes[newX][newY-1])
//					//	*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, m.Nodes[newX+1][newY])
//					//}
//					*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, m.Nodes[newX][newY-1])
//					*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, m.Nodes[newX+1][newY])
//					//2.再改变当前点索引
//					newX++
//					newY--
//					if newX == x2 && newY == y2 {
//						return
//					}
//					//已知必经点不是墙
//					nowNode = m.Nodes[newX][newY]
//					if !nowNode.IsWall() {
//						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//					}
//				}
//			}
//			//3.打印左上斜
//			//xy2
//			//    \
//			//      xy1 start
//			if dx < 0 && dy < 0 {
//				newX := x1
//				newY := y1
//				for {
//					//1.先判断当前点的 上和左 2个都是墙
//					//if !m.Nodes[newX-1][newY].IsWall() && !m.Nodes[newX][newY-1].IsWall() {
//					//	*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, m.Nodes[newX-1][newY])
//					//	*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, m.Nodes[newX][newY-1])
//					//}
//					*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, m.Nodes[newX-1][newY])
//					*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, m.Nodes[newX][newY-1])
//					//2.再改变当前点索引
//					newX--
//					newY--
//					if newX == x2 && newY == y2 {
//						return
//					}
//					//已知必经点不是墙
//					nowNode = m.Nodes[newX][newY]
//					if !nowNode.IsWall() {
//						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//					}
//				}
//			}
//			//4.打印右上斜
//			//      xy2
//			//    /
//			//xy1
//			if dx < 0 && dy > 0 {
//				newX := x1
//				newY := y1
//				for {
//					//1.先判断当前点的 上和右 2个都是墙
//					//if !m.Nodes[newX-1][newY].IsWall() && !m.Nodes[newX][newY+1].IsWall() {
//					//	*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, m.Nodes[newX-1][newY])
//					//	*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, m.Nodes[newX][newY+1])
//					//}
//					*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, m.Nodes[newX-1][newY])
//					*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, m.Nodes[newX][newY+1])
//					//2.再改变当前点索引
//					newX--
//					newY++
//					if newX == x2 && newY == y2 {
//						return
//					}
//					//已知必经点不是墙
//					nowNode = m.Nodes[newX][newY]
//					if !nowNode.IsWall() {
//						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//					}
//				}
//			}
//		}
//		//0.前面已经计算
//		//dx := x2 - x1
//		//dy := y2 - y1
//		//4.y = kx +b 的情况
//		//fmt.Println("处于y = kx +b的情况")
//		//1.计算 k 和 b
//		k, b := 0.0, 0.0
//		k = float64(dy) / float64(dx)
//		b = float64(y1) + 0.5 - (k * (float64(x1) + 0.5))
//		isYKXBRightUp := false
//		isYKXBRightDown := false
//		isYKXBLeftDown := false
//		isYKXBLeftUp := false
//		//判断方向
//		if dx > 0 && dy > 0 {
//			//1.打印右下斜
//			//xy1 start
//			//    \
//			//      xy2终点
//			//函数图右上
//			isYKXBRightUp = true
//		} else if dx > 0 && dy < 0 {
//			//2.打印左下斜
//			//      xy1 start
//			//    /
//			//xy2终点
//			//函数图右下
//			isYKXBRightDown = true
//		} else if dx < 0 && dy < 0 {
//			//3.打印左上斜
//			//xy2
//			//    \
//			//      xy1 start
//			//函数图左下
//			isYKXBLeftDown = true
//		} else if dx < 0 && dy > 0 {
//			//4.打印右上斜
//			//      xy2
//			//    /
//			//xy1
//			//函数图左上
//			isYKXBLeftUp = true
//		}
//		smallX, smallY, bigX, bigY := 0, 0, 0, 0
//		m.NowNotRepeatNodeMap[m.Nodes[x1][y1]] = 0 //判断点提前加入重复map防止被添加
//		m.NowNotRepeatNodeMap[m.Nodes[x2][y2]] = 0 //判断点提前加入重复map防止被添加
//		ok := false
//		//判断选择哪个轴
//		if absInt(dx) > absInt(dy) {
//			//选择x轴作为变量 动态Y判断左右
//			focusIntYUp := 0
//			focusButUp := false
//			focusButDown := false
//			dynamicY := 0.0
//			intDynamicY := 0
//			//判断函数图方向中,谁的索引差大就选择索引轴(X或Y轴)作为遍历去计算另一个值
//			if isYKXBRightUp {
//				//函数图右上
//				//      xy2
//				//    /
//				//xy1 start
//				smallX = x1
//				bigX = x2
//				//通用遍历左右
//				for XRight := smallX + 1; XRight <= bigX; XRight++ {
//					//0.重置当前Y与交点上下判断
//					focusButUp = false
//					focusButDown = false
//					//1.根据不断增加的X获得Y
//					dynamicY = (k * float64(XRight)) + b
//					intDynamicY = int(dynamicY)
//					//2.计算当前Y与当前Y的floor差值
//					//3.特殊情况:刚好处于交点 也就是Floor和Ceil相等
//					if math.Floor(dynamicY) == math.Ceil(dynamicY) {
//						focusButUp, focusButDown = true, true
//						focusIntYUp = intDynamicY //当前上Y索引 去掉小数
//					} else if dynamicY-math.Floor(dynamicY) < 0.01 { //focusDis必定大于0
//						focusButUp = true         //当前Y值在交点上方
//						focusIntYUp = intDynamicY //当前上Y索引
//					} else if math.Ceil(dynamicY)-dynamicY <= 0.01 { //focusDis必定大于0
//						focusButDown = true           //当前Y值在交点下方
//						focusIntYUp = intDynamicY + 1 //当前上Y索引
//					}
//					//3.如果差值小于某个数 认为碰到交点
//					if focusButUp || focusButDown {
//						//不要重复点 midNotWallNodeSliceFromX1按顺序添加
//						//必经点  连通性已确认不是墙
//						nowNode = m.Nodes[XRight-1][focusIntYUp-1] //中点左下
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						//擦边点 连通性未确认不是墙
//						nowNode = m.Nodes[XRight-1][focusIntYUp] //中点左上
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						nowNode = m.Nodes[XRight][focusIntYUp-1] //中点右下
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						//必经点 连通性已确认不是墙
//						nowNode = m.Nodes[XRight][focusIntYUp] //中点右上
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//					} else {
//						//6.不处于交点 判断int点就行
//						//必经点 连通性已确认不是墙
//						nowNode = m.Nodes[XRight-1][intDynamicY] //左
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						nowNode = m.Nodes[XRight][intDynamicY] //右
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//					}
//				}
//			} else if isYKXBRightDown {
//				//函数图右下
//				//xy1 start
//				//    \
//				//      xy2
//				smallX = x1
//				bigX = x2
//				//通用遍历左右
//				for XRight := smallX + 1; XRight <= bigX; XRight++ {
//					//0.重置当前Y与交点上下判断
//					focusButUp = false
//					focusButDown = false
//					//1.根据不断增加的X获得Y
//					dynamicY = (k * float64(XRight)) + b
//					intDynamicY = int(dynamicY)
//					//2.计算当前Y与当前Y的floor差值
//					//3.特殊情况:刚好处于交点 也就是Floor和Ceil相等
//					if math.Floor(dynamicY) == math.Ceil(dynamicY) {
//						focusButUp, focusButDown = true, true
//						focusIntYUp = intDynamicY //当前上Y索引 去掉小数
//					} else if dynamicY-math.Floor(dynamicY) < 0.01 { //focusDis必定大于0
//						focusButUp = true         //当前Y值在交点上方
//						focusIntYUp = intDynamicY //当前上Y索引
//					} else if math.Ceil(dynamicY)-dynamicY <= 0.01 { //focusDis必定大于0
//						focusButDown = true           //当前Y值在交点下方
//						focusIntYUp = intDynamicY + 1 //当前上Y索引
//					}
//					//3.如果差值小于某个数 认为碰到交点
//					if focusButUp || focusButDown {
//						//不要重复点 midNotWallNodeSliceFromX1按顺序添加
//						//必经点  连通性已确认不是墙
//						nowNode = m.Nodes[XRight-1][focusIntYUp] //中点左上
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						//擦边点 连通性未确认不是墙
//						nowNode = m.Nodes[XRight][focusIntYUp] //中点右上
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						nowNode = m.Nodes[XRight-1][focusIntYUp-1] //中点左下
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						//必经点 连通性已确认不是墙
//						nowNode = m.Nodes[XRight][focusIntYUp-1] //中点右下
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//					} else {
//						//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
//						//6.不处于交点 判断int点就行
//						//必经点 连通性已确认不是墙
//						nowNode = m.Nodes[XRight-1][intDynamicY] //左
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						nowNode = m.Nodes[XRight][intDynamicY] //右
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//					}
//				}
//			} else if isYKXBLeftDown {
//				//函数图左下
//				//     xy1 start
//				//    /
//				//xy2
//				smallX = x2
//				bigX = x1
//				//通用遍历左右
//				for XRight := bigX; XRight > smallX; XRight-- {
//					//0.重置当前Y与交点上下判断
//					focusButUp = false
//					focusButDown = false
//					//1.根据不断增加的X获得Y
//					dynamicY = (k * float64(XRight)) + b
//					intDynamicY = int(dynamicY)
//					//2.计算当前Y与当前Y的floor差值
//					//3.特殊情况:刚好处于交点 也就是Floor和Ceil相等
//					if math.Floor(dynamicY) == math.Ceil(dynamicY) {
//						focusButUp, focusButDown = true, true
//						focusIntYUp = intDynamicY //当前上Y索引 去掉小数
//					} else if dynamicY-math.Floor(dynamicY) < 0.01 { //focusDis必定大于0
//						focusButUp = true         //当前Y值在交点上方
//						focusIntYUp = intDynamicY //当前上Y索引
//					} else if math.Ceil(dynamicY)-dynamicY <= 0.01 { //focusDis必定大于0
//						focusButDown = true           //当前Y值在交点下方
//						focusIntYUp = intDynamicY + 1 //当前上Y索引
//					}
//					//3.如果差值小于某个数 认为碰到交点
//					if focusButUp || focusButDown {
//						//4.判断必经点 擦边点 注意：不同方向必经点不一样
//						//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][focusIntYUp-1], m.Nodes[XRight][focusIntYUp],
//						//	m.Nodes[XRight-1][focusIntYUp], m.Nodes[XRight][focusIntYUp-1])
//						//5.如果当前方向是 /
//						//不要重复点 midNotWallNodeSliceFromX1按顺序添加
//						//必经点 连通性已确认不是墙
//						nowNode = m.Nodes[XRight][focusIntYUp] //中点右上
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						//擦边点 连通性未确认不是墙
//						nowNode = m.Nodes[XRight-1][focusIntYUp] //中点左上
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						nowNode = m.Nodes[XRight][focusIntYUp-1] //中点右下
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						//必经点  连通性已确认不是墙
//						nowNode = m.Nodes[XRight-1][focusIntYUp-1] //中点左下
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//					} else {
//						//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
//						//6.不处于交点 判断int点就行
//						//必经点 连通性已确认不是墙
//						nowNode = m.Nodes[XRight][intDynamicY] //右
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						nowNode = m.Nodes[XRight-1][intDynamicY] //左
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//					}
//				}
//			} else if isYKXBLeftUp {
//				//函数图左上
//				//xy2
//				//    \
//				//    xy1 start
//				smallX = x2
//				bigX = x1
//				//通用遍历左右
//				for XRight := bigX; XRight > smallX; XRight-- {
//					//0.重置当前Y与交点上下判断
//					focusButUp = false
//					focusButDown = false
//					//1.根据不断增加的X获得Y
//					dynamicY = (k * float64(XRight)) + b
//					intDynamicY = int(dynamicY)
//					//2.计算当前Y与当前Y的floor差值
//					//3.特殊情况:刚好处于交点 也就是Floor和Ceil相等
//					if math.Floor(dynamicY) == math.Ceil(dynamicY) {
//						focusButUp, focusButDown = true, true
//						focusIntYUp = intDynamicY //当前上Y索引 去掉小数
//					} else if dynamicY-math.Floor(dynamicY) < 0.01 { //focusDis必定大于0
//						focusButUp = true         //当前Y值在交点上方
//						focusIntYUp = intDynamicY //当前上Y索引
//					} else if math.Ceil(dynamicY)-dynamicY <= 0.01 { //focusDis必定大于0
//						focusButDown = true           //当前Y值在交点下方
//						focusIntYUp = intDynamicY + 1 //当前上Y索引
//					}
//					//3.如果差值小于某个数 认为碰到交点
//					if focusButUp || focusButDown {
//						//4.判断必经点 擦边点 注意：不同方向必经点不一样
//						//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][focusIntYUp-1], m.Nodes[XRight][focusIntYUp],
//						//	m.Nodes[XRight-1][focusIntYUp], m.Nodes[XRight][focusIntYUp-1])
//						//5.如果当前方向是 /
//						//不要重复点 midNotWallNodeSliceFromX1按顺序添加
//						//必经点 连通性已确认不是墙
//						nowNode = m.Nodes[XRight][focusIntYUp-1] //中点右下
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						//擦边点 连通性未确认不是墙
//						nowNode = m.Nodes[XRight-1][focusIntYUp-1] //中点左下
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						nowNode = m.Nodes[XRight][focusIntYUp] //中点右上
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						//必经点  连通性已确认不是墙
//						nowNode = m.Nodes[XRight-1][focusIntYUp] //中点左上
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//					} else {
//						//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
//						//6.不处于交点 判断int点就行
//						//必经点 连通性已确认不是墙
//						nowNode = m.Nodes[XRight][intDynamicY] //右
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						nowNode = m.Nodes[XRight-1][intDynamicY] //左
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//					}
//				}
//			}
//		} else { //dx < dy
//			//选择y轴作为变量 判断上下
//			focusIntXLeft := 0
//			focusButLeft := false
//			focusButRight := false
//			dynamicX := 0.0
//			intDynamicX := 0
//			//判断函数图方向中,谁的索引差大就选择索引轴(X或Y轴)作为遍历去计算另一个值
//			if isYKXBRightUp {
//				//函数图右上
//				//      xy2
//				//    /
//				//xy1 start
//				smallY = y1
//				bigY = y2
//				//通用遍历上下
//				for YUp := smallY + 1; YUp <= bigY; YUp++ {
//					//0.重置当前Y与交点上下判断
//					focusButLeft = false
//					focusButRight = false
//					//1.根据不断增加的Y获得X
//					dynamicX = (float64(YUp) - b) / k
//					intDynamicX = int(dynamicX)
//					//2.计算当前X与当前X的floor差值
//					if math.Ceil(dynamicX) == math.Floor(dynamicX) {
//						focusButLeft, focusButRight = true, true
//						focusIntXLeft = intDynamicX - 1 //当前左X索引
//					} else if math.Ceil(dynamicX)-dynamicX < 0.01 { //focusDis必定大于0
//						focusButLeft = true         //当前X值在交点左
//						focusIntXLeft = intDynamicX //当前左X索引
//					} else if dynamicX-math.Floor(dynamicX) < 0.01 { //focusDis必定大于0
//						focusButRight = true            //当前X值在交点右
//						focusIntXLeft = intDynamicX - 1 //当前左X索引
//					}
//					//3.如果差值小于某个数或相等 认为碰到交点
//					if focusButLeft || focusButRight {
//						//不要重复点 midNotWallNodeSliceFromX1按顺序添加
//						//必经点  连通性已确认不是墙
//						nowNode = m.Nodes[focusIntXLeft][YUp-1] //中点左下
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						//擦边点 连通性未确认不是墙
//						nowNode = m.Nodes[focusIntXLeft+1][YUp-1] //中点右下
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						nowNode = m.Nodes[focusIntXLeft][YUp] //中点左上
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						//必经点 连通性已确认不是墙
//						nowNode = m.Nodes[focusIntXLeft+1][YUp] //中点右上
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//					} else {
//						//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
//						//6.不处于交点 判断int点就行
//						//必经点 连通性已确认不是墙
//						nowNode = m.Nodes[intDynamicX][YUp-1] //下
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						nowNode = m.Nodes[intDynamicX][YUp] //上
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//					}
//				}
//			} else if isYKXBRightDown {
//				//函数图右下
//				//xy1 start
//				//    \
//				//      xy2
//				smallY = y2
//				bigY = y1
//				//通用遍历上下
//				for YUp := bigY; YUp > smallY; YUp-- {
//					//0.重置当前Y与交点上下判断
//					focusButLeft = false
//					focusButRight = false
//					//1.根据不断增加的Y获得X
//					dynamicX = (float64(YUp) - b) / k
//					intDynamicX = int(dynamicX)
//					//2.计算当前X与当前X的floor差值
//					if math.Ceil(dynamicX) == math.Floor(dynamicX) {
//						focusButLeft, focusButRight = true, true
//						focusIntXLeft = intDynamicX - 1 //当前左X索引
//					} else if math.Ceil(dynamicX)-dynamicX < 0.01 { //focusDis必定大于0
//						focusButLeft = true         //当前X值在交点左
//						focusIntXLeft = intDynamicX //当前左X索引
//					} else if dynamicX-math.Floor(dynamicX) < 0.01 { //focusDis必定大于0
//						focusButRight = true            //当前X值在交点右
//						focusIntXLeft = intDynamicX - 1 //当前左X索引
//					}
//					//3.如果差值小于某个数或相等 认为碰到交点
//					if focusButLeft || focusButRight {
//						//不要重复点 midNotWallNodeSliceFromX1按顺序添加
//						//必经点  连通性已确认不是墙
//						nowNode = m.Nodes[focusIntXLeft][YUp] //中点左上
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						//擦边点 连通性未确认不是墙
//						nowNode = m.Nodes[focusIntXLeft+1][YUp] //中点右上
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						nowNode = m.Nodes[focusIntXLeft][YUp-1] //中点左下
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						//必经点 连通性已确认不是墙
//						nowNode = m.Nodes[focusIntXLeft+1][YUp-1] //中点右下
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//					} else {
//						//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
//						//6.不处于交点 判断int点就行
//						//必经点 连通性已确认不是墙
//						nowNode = m.Nodes[intDynamicX][YUp] //上
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						nowNode = m.Nodes[intDynamicX][YUp-1] //下
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//					}
//				}
//			} else if isYKXBLeftDown {
//				//函数图左下
//				//     xy1 start
//				//    /
//				//xy2终点
//				smallY = y2
//				bigY = y1
//				//通用遍历上下
//				for YUp := bigY; YUp > smallY; YUp-- {
//					//0.重置当前Y与交点上下判断
//					focusButLeft = false
//					focusButRight = false
//					//1.根据不断增加的Y获得X
//					dynamicX = (float64(YUp) - b) / k
//					intDynamicX = int(dynamicX)
//					//2.计算当前X与当前X的floor差值
//					if math.Ceil(dynamicX) == math.Floor(dynamicX) {
//						focusButLeft, focusButRight = true, true
//						focusIntXLeft = intDynamicX - 1 //当前左X索引
//					} else if math.Ceil(dynamicX)-dynamicX < 0.01 { //focusDis必定大于0
//						focusButLeft = true         //当前X值在交点左
//						focusIntXLeft = intDynamicX //当前左X索引
//					} else if dynamicX-math.Floor(dynamicX) < 0.01 { //focusDis必定大于0
//						focusButRight = true            //当前X值在交点右
//						focusIntXLeft = intDynamicX - 1 //当前左X索引
//					}
//					//3.如果差值小于某个数或相等 认为碰到交点
//					if focusButLeft || focusButRight {
//						//4.判断必经点 擦边点 注意：不同方向必经点不一样
//						//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][focusIntYUp-1], m.Nodes[XRight][focusIntYUp],
//						//	m.Nodes[XRight-1][focusIntYUp], m.Nodes[XRight][focusIntYUp-1])
//						//5.如果当前方向是 /
//						//不要重复点 midNotWallNodeSliceFromX1按顺序添加
//						//必经点  连通性已确认不是墙
//						nowNode = m.Nodes[focusIntXLeft+1][YUp] //中点右上
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						//擦边点 连通性未确认不是墙
//						nowNode = m.Nodes[focusIntXLeft][YUp] //中点左上
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						nowNode = m.Nodes[focusIntXLeft+1][YUp-1] //中点右下
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						//必经点 连通性已确认不是墙
//						nowNode = m.Nodes[focusIntXLeft][YUp-1] //中点左下
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//					} else {
//						//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
//						//6.不处于交点 判断int点就行
//						//必经点 连通性已确认不是墙
//						nowNode = m.Nodes[intDynamicX][YUp] //上
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						nowNode = m.Nodes[intDynamicX][YUp-1] //下
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//					}
//				}
//			} else if isYKXBLeftUp {
//				//函数图左上
//				//xy2
//				//    \
//				//    xy1 start
//				smallY = y1
//				bigY = y2
//				//通用遍历上下
//				for YUp := smallY + 1; YUp <= bigY; YUp++ {
//					//0.重置当前Y与交点上下判断
//					focusButLeft = false
//					focusButRight = false
//					//1.根据不断增加的Y获得X
//					dynamicX = (float64(YUp) - b) / k
//					intDynamicX = int(dynamicX)
//					//2.计算当前X与当前X的floor差值
//					if math.Ceil(dynamicX) == math.Floor(dynamicX) {
//						focusButLeft, focusButRight = true, true
//						focusIntXLeft = intDynamicX - 1 //当前左X索引
//					} else if math.Ceil(dynamicX)-dynamicX < 0.01 { //focusDis必定大于0
//						focusButLeft = true         //当前X值在交点左
//						focusIntXLeft = intDynamicX //当前左X索引
//					} else if dynamicX-math.Floor(dynamicX) < 0.01 { //focusDis必定大于0
//						focusButRight = true            //当前X值在交点右
//						focusIntXLeft = intDynamicX - 1 //当前左X索引
//					}
//					//3.如果差值小于某个数或相等 认为碰到交点
//					if focusButLeft || focusButRight {
//						//4.判断必经点 擦边点 注意：不同方向必经点不一样
//						//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][focusIntYUp-1], m.Nodes[XRight][focusIntYUp],
//						//	m.Nodes[XRight-1][focusIntYUp], m.Nodes[XRight][focusIntYUp-1])
//						//5.如果当前方向是 /
//						//不要重复点 midNotWallNodeSliceFromX1按顺序添加
//						//必经点  连通性已确认不是墙
//						nowNode = m.Nodes[focusIntXLeft+1][YUp-1] //中点右下
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						//擦边点 连通性未确认不是墙
//						nowNode = m.Nodes[focusIntXLeft+1][YUp] //中点右上
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						nowNode = m.Nodes[focusIntXLeft][YUp-1] //中点左下
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						//必经点 连通性已确认不是墙
//						nowNode = m.Nodes[focusIntXLeft][YUp] //中点左上
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//					} else {
//						//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
//						//6.不处于交点 判断int点就行
//						//必经点 连通性已确认不是墙
//						nowNode = m.Nodes[intDynamicX][YUp-1] //下
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//						nowNode = m.Nodes[intDynamicX][YUp] //上
//						if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
//							*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
//							m.NowNotRepeatNodeMap[nowNode] = 0
//						}
//					}
//				}
//			}
//		}
//
// }
const epsilon = 1e-9 // 比 float64 精度稍大
// -------------------------------------------------- 后处理相关

// GetMidNotObstacleFromX1ToX2ForStep2 生成两点之间像素点(不要障碍物) 两点已确认可以相连---水平垂直45度斜向:必经点 y=kx+b:必经点+擦边点 默认参数x1, y1, x2, y2都在地图之内
func (m *Map) GetMidNotObstacleFromX1ToX2ForStep2(x1, y1, x2, y2 int, OriginalMidNode []*Node, midNotWallNodeSliceFromX1 *[]*Node) {
	//每次获得中间像素点之前 先重置map和slice 并提前设置替换点已存在 防止最后替换同一个点
	clear(m.NowNotRepeatNodeMap)
	*midNotWallNodeSliceFromX1 = (*midNotWallNodeSliceFromX1)[:0]
	for i := 0; i < len(OriginalMidNode); i++ {
		if OriginalMidNode[i] != nil {
			m.NowNotRepeatNodeMap[OriginalMidNode[i]] = 0
		}
	}
	//这一步 传入指针切片，内部把不是墙的节点添加完毕后传给外部
	var nowNode *Node
	//judgeNodeSlice := make([]*Node, 0, 20)
	//1.直线同1行
	if x1 == x2 {
		if y1 < y2 {
			//1start  2
			for newY := y1 + 1; newY < y2; newY++ {
				//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[x1][newY])
				//如果直线障碍点有1个是墙 直接返回false
				//if m.Nodes[x1][newY].IsWall()  {//如果中间点不是墙 本来就没有起点终点 不用判断当前是起点终点
				//	return false
				//}
				nowNode = m.Nodes[x1][newY]
				if !nowNode.IsWall() {
					*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode) //这个方法默认之前判断可以相连 默认不是墙 本来就没有起点终点 不用判断当前是起点终点
				}
			}
		} else {
			//2 1start
			for newY := y1 - 1; newY > y2; newY-- {
				//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[x1][newY])
				//如果直线障碍点有1个是墙 直接返回false
				//if m.Nodes[x1][newY].IsWall()  {//如果中间点不是墙 本来就没有起点终点 不用判断当前是起点终点
				//	return false
				//}
				nowNode = m.Nodes[x1][newY]
				if !nowNode.IsWall() {
					*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode) //这个方法默认之前判断可以相连 默认不是墙 本来就没有起点终点 不用判断当前是起点终点
				}
			}
		}
		//记得退出
		return
	}
	//2.直线同1列
	if y1 == y2 {
		if x1 < x2 {
			//1 Small start
			//2 big
			for newX := x1 + 1; newX < x2; newX++ {
				//如果直线障碍点有1个是墙 直接返回false
				//if m.Nodes[newX][y1].IsWall() {
				//	return false
				//}
				nowNode = m.Nodes[newX][y1]
				if !nowNode.IsWall() {
					*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode) //这个方法默认之前判断可以相连 默认不是墙 本来就没有起点终点 不用判断当前是起点终点
				}
			}
		} else {
			//2 Small
			//1 big start
			for newX := x1 - 1; newX > x2; newX-- {
				//如果直线障碍点有1个是墙 直接返回false
				//if m.Nodes[newX][y1].IsWall() {
				//	return false
				//}
				nowNode = m.Nodes[newX][y1]
				if !nowNode.IsWall() {
					*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode) //这个方法默认之前判断可以相连 默认不是墙 本来就没有起点终点 不用判断当前是起点终点
				}
			}
		}
		//记得退出
		return
	}
	//0.先计算dx dy,用后面的点减去前面的点
	dx := x2 - x1
	dy := y2 - y1
	//3.处于标准的正方形对角线2点 判断是不是标准正方形 斜向45度 只选择必经点 只选择必经点 只选择必经点 只选择必经点!!!
	if dx == dy || dx == -dy {
		newX := x1
		newY := y1
		//1.打印右下斜
		//xy1 start
		//    \
		//      xy2终点
		if dx > 0 && dy > 0 {
			for {
				newX++
				newY++
				if newX == x2 && newY == y2 {
					return
				}
				//已知必经点不是墙
				nowNode = m.Nodes[newX][newY]
				if !nowNode.IsWall() {
					*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
				}
				//擦边点判断是否是墙 一定在界内
			}
		}
		//2.打印左下斜
		//      xy1 start
		//    /
		//xy2终点
		if dx > 0 && dy < 0 {
			for {
				newX++
				newY--
				if newX == x2 && newY == y2 {
					return
				}
				//已知必经点不是墙
				nowNode = m.Nodes[newX][newY]
				if !nowNode.IsWall() {
					*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
				}
			}
		}
		//3.打印左上斜
		//xy2
		//    \
		//      xy1 start
		if dx < 0 && dy < 0 {
			for {
				newX--
				newY--
				if newX == x2 && newY == y2 {
					return
				}
				//已知必经点不是墙
				nowNode = m.Nodes[newX][newY]
				if !nowNode.IsWall() {
					*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
				}
			}
		}
		//4.打印右上斜
		//      xy2
		//    /
		//xy1
		if dx < 0 && dy > 0 {
			for {
				newX--
				newY++
				if newX == x2 && newY == y2 {
					return
				}
				//已知必经点不是墙
				nowNode = m.Nodes[newX][newY]
				if !nowNode.IsWall() {
					*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
				}
			}
		}
	}
	//0.前面已经计算
	//dx := x2 - x1
	//dy := y2 - y1
	//4.y = kx +b 的情况
	//fmt.Println("处于y = kx +b的情况")
	//1.计算 k 和 b
	k, b := 0.0, 0.0
	k = float64(dy) / float64(dx)
	b = float64(y1) + 0.5 - (k * (float64(x1) + 0.5))
	isYKXBRightUp := false
	isYKXBRightDown := false
	isYKXBLeftDown := false
	isYKXBLeftUp := false
	//判断方向
	if dx > 0 && dy > 0 {
		//1.打印右下斜
		//xy1 start
		//    \
		//      xy2终点
		//函数图右上
		isYKXBRightUp = true
	} else if dx > 0 && dy < 0 {
		//2.打印左下斜
		//      xy1 start
		//    /
		//xy2终点
		//函数图右下
		isYKXBRightDown = true
	} else if dx < 0 && dy < 0 {
		//3.打印左上斜
		//xy2
		//    \
		//      xy1 start
		//函数图左下
		isYKXBLeftDown = true
	} else if dx < 0 && dy > 0 {
		//4.打印右上斜
		//      xy2
		//    /
		//xy1
		//函数图左上
		isYKXBLeftUp = true
	}
	smallX, smallY, bigX, bigY := 0, 0, 0, 0
	m.NowNotRepeatNodeMap[m.Nodes[x1][y1]] = 0 //判断点提前加入重复map防止被添加
	m.NowNotRepeatNodeMap[m.Nodes[x2][y2]] = 0 //判断点提前加入重复map防止被添加
	ok := false
	//判断选择哪个轴
	if absInt(dx) > absInt(dy) {
		//选择x轴作为变量 动态Y判断左右
		focusIntYUp := 0
		focusButUp := false
		focusButDown := false
		dynamicY := 0.0
		intDynamicY := 0
		//判断函数图方向中,谁的索引差大就选择索引轴(X或Y轴)作为遍历去计算另一个值
		if isYKXBRightUp {
			//函数图右上
			//      xy2
			//    /
			//xy1 start
			smallX = x1
			bigX = x2
			//通用遍历左右
			for XRight := smallX + 1; XRight <= bigX; XRight++ {
				//0.重置当前Y与交点上下判断
				focusButUp = false
				focusButDown = false
				//1.根据不断增加的X获得Y
				dynamicY = (k * float64(XRight)) + b
				intDynamicY = int(dynamicY)
				//2.计算当前Y与当前Y的floor差值
				//3.特殊情况:刚好处于交点 也就是Floor和Ceil相等
				if math.Floor(dynamicY) == math.Ceil(dynamicY) {
					focusButUp, focusButDown = true, true
					focusIntYUp = intDynamicY //当前上Y索引 去掉小数
				} else if dynamicY-math.Floor(dynamicY) < 0.01 { //focusDis必定大于0
					focusButUp = true         //当前Y值在交点上方
					focusIntYUp = intDynamicY //当前上Y索引
				} else if math.Ceil(dynamicY)-dynamicY <= 0.01 { //focusDis必定大于0
					focusButDown = true           //当前Y值在交点下方
					focusIntYUp = intDynamicY + 1 //当前上Y索引
				}
				//3.如果差值小于某个数 认为碰到交点
				if focusButUp || focusButDown {
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[XRight-1][focusIntYUp-1] //中点左下
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					//擦边点 连通性未确认不是墙
					nowNode = m.Nodes[XRight-1][focusIntYUp] //中点左上
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					nowNode = m.Nodes[XRight][focusIntYUp-1] //中点右下
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight][focusIntYUp] //中点右上
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
				} else {
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight-1][intDynamicY] //左
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					nowNode = m.Nodes[XRight][intDynamicY] //右
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
				}
			}
		} else if isYKXBRightDown {
			//函数图右下
			//xy1 start
			//    \
			//      xy2
			smallX = x1
			bigX = x2
			//通用遍历左右
			for XRight := smallX + 1; XRight <= bigX; XRight++ {
				//0.重置当前Y与交点上下判断
				focusButUp = false
				focusButDown = false
				//1.根据不断增加的X获得Y
				dynamicY = (k * float64(XRight)) + b
				intDynamicY = int(dynamicY)
				//2.计算当前Y与当前Y的floor差值
				//3.特殊情况:刚好处于交点 也就是Floor和Ceil相等
				if math.Floor(dynamicY) == math.Ceil(dynamicY) {
					focusButUp, focusButDown = true, true
					focusIntYUp = intDynamicY //当前上Y索引 去掉小数
				} else if dynamicY-math.Floor(dynamicY) < 0.01 { //focusDis必定大于0
					focusButUp = true         //当前Y值在交点上方
					focusIntYUp = intDynamicY //当前上Y索引
				} else if math.Ceil(dynamicY)-dynamicY <= 0.01 { //focusDis必定大于0
					focusButDown = true           //当前Y值在交点下方
					focusIntYUp = intDynamicY + 1 //当前上Y索引
				}
				//3.如果差值小于某个数 认为碰到交点
				if focusButUp || focusButDown {
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[XRight-1][focusIntYUp] //中点左上
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					//擦边点 连通性未确认不是墙
					nowNode = m.Nodes[XRight][focusIntYUp] //中点右上
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					nowNode = m.Nodes[XRight-1][focusIntYUp-1] //中点左下
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight][focusIntYUp-1] //中点右下
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight-1][intDynamicY] //左
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					nowNode = m.Nodes[XRight][intDynamicY] //右
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
				}
			}
		} else if isYKXBLeftDown {
			//函数图左下
			//     xy1 start
			//    /
			//xy2
			smallX = x2
			bigX = x1
			//通用遍历左右
			for XRight := bigX; XRight > smallX; XRight-- {
				//0.重置当前Y与交点上下判断
				focusButUp = false
				focusButDown = false
				//1.根据不断增加的X获得Y
				dynamicY = (k * float64(XRight)) + b
				intDynamicY = int(dynamicY)
				//2.计算当前Y与当前Y的floor差值
				//3.特殊情况:刚好处于交点 也就是Floor和Ceil相等
				if math.Floor(dynamicY) == math.Ceil(dynamicY) {
					focusButUp, focusButDown = true, true
					focusIntYUp = intDynamicY //当前上Y索引 去掉小数
				} else if dynamicY-math.Floor(dynamicY) < 0.01 { //focusDis必定大于0
					focusButUp = true         //当前Y值在交点上方
					focusIntYUp = intDynamicY //当前上Y索引
				} else if math.Ceil(dynamicY)-dynamicY <= 0.01 { //focusDis必定大于0
					focusButDown = true           //当前Y值在交点下方
					focusIntYUp = intDynamicY + 1 //当前上Y索引
				}
				//3.如果差值小于某个数 认为碰到交点
				if focusButUp || focusButDown {
					//4.判断必经点 擦边点 注意：不同方向必经点不一样
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][focusIntYUp-1], m.Nodes[XRight][focusIntYUp],
					//	m.Nodes[XRight-1][focusIntYUp], m.Nodes[XRight][focusIntYUp-1])
					//5.如果当前方向是 /
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight][focusIntYUp] //中点右上
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					//擦边点 连通性未确认不是墙
					nowNode = m.Nodes[XRight-1][focusIntYUp] //中点左上
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					nowNode = m.Nodes[XRight][focusIntYUp-1] //中点右下
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[XRight-1][focusIntYUp-1] //中点左下
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight][intDynamicY] //右
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					nowNode = m.Nodes[XRight-1][intDynamicY] //左
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
				}
			}
		} else if isYKXBLeftUp {
			//函数图左上
			//xy2
			//    \
			//    xy1 start
			smallX = x2
			bigX = x1
			//通用遍历左右
			for XRight := bigX; XRight > smallX; XRight-- {
				//0.重置当前Y与交点上下判断
				focusButUp = false
				focusButDown = false
				//1.根据不断增加的X获得Y
				dynamicY = (k * float64(XRight)) + b
				intDynamicY = int(dynamicY)
				//2.计算当前Y与当前Y的floor差值
				//3.特殊情况:刚好处于交点 也就是Floor和Ceil相等
				if math.Floor(dynamicY) == math.Ceil(dynamicY) {
					focusButUp, focusButDown = true, true
					focusIntYUp = intDynamicY //当前上Y索引 去掉小数
				} else if dynamicY-math.Floor(dynamicY) < 0.01 { //focusDis必定大于0
					focusButUp = true         //当前Y值在交点上方
					focusIntYUp = intDynamicY //当前上Y索引
				} else if math.Ceil(dynamicY)-dynamicY <= 0.01 { //focusDis必定大于0
					focusButDown = true           //当前Y值在交点下方
					focusIntYUp = intDynamicY + 1 //当前上Y索引
				}
				//3.如果差值小于某个数 认为碰到交点
				if focusButUp || focusButDown {
					//4.判断必经点 擦边点 注意：不同方向必经点不一样
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][focusIntYUp-1], m.Nodes[XRight][focusIntYUp],
					//	m.Nodes[XRight-1][focusIntYUp], m.Nodes[XRight][focusIntYUp-1])
					//5.如果当前方向是 /
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight][focusIntYUp-1] //中点右下
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					//擦边点 连通性未确认不是墙
					nowNode = m.Nodes[XRight-1][focusIntYUp-1] //中点左下
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					nowNode = m.Nodes[XRight][focusIntYUp] //中点右上
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[XRight-1][focusIntYUp] //中点左上
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[XRight][intDynamicY] //右
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					nowNode = m.Nodes[XRight-1][intDynamicY] //左
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
				}
			}
		}
	} else { //dx < dy
		//选择y轴作为变量 判断上下
		focusIntXLeft := 0
		focusButLeft := false
		focusButRight := false
		dynamicX := 0.0
		intDynamicX := 0
		//判断函数图方向中,谁的索引差大就选择索引轴(X或Y轴)作为遍历去计算另一个值
		if isYKXBRightUp {
			//函数图右上
			//      xy2
			//    /
			//xy1 start
			smallY = y1
			bigY = y2
			//通用遍历上下
			for YUp := smallY + 1; YUp <= bigY; YUp++ {
				//0.重置当前Y与交点上下判断
				focusButLeft = false
				focusButRight = false
				//1.根据不断增加的Y获得X
				dynamicX = (float64(YUp) - b) / k
				intDynamicX = int(dynamicX)
				//2.计算当前X与当前X的floor差值
				if math.Ceil(dynamicX) == math.Floor(dynamicX) {
					focusButLeft, focusButRight = true, true
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				} else if math.Ceil(dynamicX)-dynamicX < 0.01 { //focusDis必定大于0
					focusButLeft = true         //当前X值在交点左
					focusIntXLeft = intDynamicX //当前左X索引
				} else if dynamicX-math.Floor(dynamicX) < 0.01 { //focusDis必定大于0
					focusButRight = true            //当前X值在交点右
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				}
				//3.如果差值小于某个数或相等 认为碰到交点
				if focusButLeft || focusButRight {
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft][YUp-1] //中点左下
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					//擦边点 连通性未确认不是墙
					nowNode = m.Nodes[focusIntXLeft+1][YUp-1] //中点右下
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					nowNode = m.Nodes[focusIntXLeft][YUp] //中点左上
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft+1][YUp] //中点右上
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[intDynamicX][YUp-1] //下
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					nowNode = m.Nodes[intDynamicX][YUp] //上
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
				}
			}
		} else if isYKXBRightDown {
			//函数图右下
			//xy1 start
			//    \
			//      xy2
			smallY = y2
			bigY = y1
			//通用遍历上下
			for YUp := bigY; YUp > smallY; YUp-- {
				//0.重置当前Y与交点上下判断
				focusButLeft = false
				focusButRight = false
				//1.根据不断增加的Y获得X
				dynamicX = (float64(YUp) - b) / k
				intDynamicX = int(dynamicX)
				//2.计算当前X与当前X的floor差值
				if math.Ceil(dynamicX) == math.Floor(dynamicX) {
					focusButLeft, focusButRight = true, true
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				} else if math.Ceil(dynamicX)-dynamicX < 0.01 { //focusDis必定大于0
					focusButLeft = true         //当前X值在交点左
					focusIntXLeft = intDynamicX //当前左X索引
				} else if dynamicX-math.Floor(dynamicX) < 0.01 { //focusDis必定大于0
					focusButRight = true            //当前X值在交点右
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				}
				//3.如果差值小于某个数或相等 认为碰到交点
				if focusButLeft || focusButRight {
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft][YUp] //中点左上
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					//擦边点 连通性未确认不是墙
					nowNode = m.Nodes[focusIntXLeft+1][YUp] //中点右上
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					nowNode = m.Nodes[focusIntXLeft][YUp-1] //中点左下
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft+1][YUp-1] //中点右下
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[intDynamicX][YUp] //上
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					nowNode = m.Nodes[intDynamicX][YUp-1] //下
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
				}
			}
		} else if isYKXBLeftDown {
			//函数图左下
			//     xy1 start
			//    /
			//xy2终点
			smallY = y2
			bigY = y1
			//通用遍历上下
			for YUp := bigY; YUp > smallY; YUp-- {
				//0.重置当前Y与交点上下判断
				focusButLeft = false
				focusButRight = false
				//1.根据不断增加的Y获得X
				dynamicX = (float64(YUp) - b) / k
				intDynamicX = int(dynamicX)
				//2.计算当前X与当前X的floor差值
				if math.Ceil(dynamicX) == math.Floor(dynamicX) {
					focusButLeft, focusButRight = true, true
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				} else if math.Ceil(dynamicX)-dynamicX < 0.01 { //focusDis必定大于0
					focusButLeft = true         //当前X值在交点左
					focusIntXLeft = intDynamicX //当前左X索引
				} else if dynamicX-math.Floor(dynamicX) < 0.01 { //focusDis必定大于0
					focusButRight = true            //当前X值在交点右
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				}
				//3.如果差值小于某个数或相等 认为碰到交点
				if focusButLeft || focusButRight {
					//4.判断必经点 擦边点 注意：不同方向必经点不一样
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][focusIntYUp-1], m.Nodes[XRight][focusIntYUp],
					//	m.Nodes[XRight-1][focusIntYUp], m.Nodes[XRight][focusIntYUp-1])
					//5.如果当前方向是 /
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft+1][YUp] //中点右上
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					//擦边点 连通性未确认不是墙
					nowNode = m.Nodes[focusIntXLeft][YUp] //中点左上
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					nowNode = m.Nodes[focusIntXLeft+1][YUp-1] //中点右下
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft][YUp-1] //中点左下
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[intDynamicX][YUp] //上
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					nowNode = m.Nodes[intDynamicX][YUp-1] //下
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
				}
			}
		} else if isYKXBLeftUp {
			//函数图左上
			//xy2
			//    \
			//    xy1 start
			smallY = y1
			bigY = y2
			//通用遍历上下
			for YUp := smallY + 1; YUp <= bigY; YUp++ {
				//0.重置当前Y与交点上下判断
				focusButLeft = false
				focusButRight = false
				//1.根据不断增加的Y获得X
				dynamicX = (float64(YUp) - b) / k
				intDynamicX = int(dynamicX)
				//2.计算当前X与当前X的floor差值
				if math.Ceil(dynamicX) == math.Floor(dynamicX) {
					focusButLeft, focusButRight = true, true
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				} else if math.Ceil(dynamicX)-dynamicX < 0.01 { //focusDis必定大于0
					focusButLeft = true         //当前X值在交点左
					focusIntXLeft = intDynamicX //当前左X索引
				} else if dynamicX-math.Floor(dynamicX) < 0.01 { //focusDis必定大于0
					focusButRight = true            //当前X值在交点右
					focusIntXLeft = intDynamicX - 1 //当前左X索引
				}
				//3.如果差值小于某个数或相等 认为碰到交点
				if focusButLeft || focusButRight {
					//4.判断必经点 擦边点 注意：不同方向必经点不一样
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][focusIntYUp-1], m.Nodes[XRight][focusIntYUp],
					//	m.Nodes[XRight-1][focusIntYUp], m.Nodes[XRight][focusIntYUp-1])
					//5.如果当前方向是 /
					//不要重复点 midNotWallNodeSliceFromX1按顺序添加
					//必经点  连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft+1][YUp-1] //中点右下
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					//擦边点 连通性未确认不是墙
					nowNode = m.Nodes[focusIntXLeft+1][YUp] //中点右上
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					nowNode = m.Nodes[focusIntXLeft][YUp-1] //中点左下
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[focusIntXLeft][YUp] //中点左上
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
				} else {
					//judgeNodeSlice = append(judgeNodeSlice, m.Nodes[XRight-1][int(dynamicY)], m.Nodes[XRight][int(dynamicY)])
					//6.不处于交点 判断int点就行
					//必经点 连通性已确认不是墙
					nowNode = m.Nodes[intDynamicX][YUp-1] //下
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
					nowNode = m.Nodes[intDynamicX][YUp] //上
					if _, ok = m.NowNotRepeatNodeMap[nowNode]; !ok && !nowNode.IsWall() {
						*midNotWallNodeSliceFromX1 = append(*midNotWallNodeSliceFromX1, nowNode)
						m.NowNotRepeatNodeMap[nowNode] = 0
					}
				}
			}
		}
	}

}

// PrintSM 输出直线障碍判断经过的判断点
func (m *Map) PrintSM() {
	fmt.Println("\nPrintSM Mid：")
	findFlag := false
	for i := 0; i < len(m.Nodes); i++ {
		for j := 0; j < len(m.Nodes[i]); j++ {
			if m.Nodes[i][j].IsWall() {
				fmt.Print("!")
			} else {
				// 检查是否找到目标切片点
				for k := 0; k < len(m.SMNodeSlice); k++ {
					if m.SMNodeSlice[k] == m.Nodes[i][j] {
						findFlag = true
						break
					}
				}
				//
				if findFlag {
					fmt.Print("#") // #代表经过的判断点(像素点)
				} else {
					fmt.Print(".")
				}
				findFlag = false
			}
		}
		fmt.Println()
	}
}

// 第一步 :弗洛伊德 正向迭代 + 反向迭代
func (m *Map) jpsSmoothPathNewStep1() {
	//1.当前点一定大于等于3个点 因为已经像素障碍判断过(可以擦边) --等于3个点的时候直接退出赋值resultNode
	//2.假如当前是ABCDEFGHI点
	//A-BC A不可以到D,记录C点,
	//从C开始,C可以-DE,C不可以F,记录E点
	//从E开始,E可以=FG,E不可以H,记录G点
	//从G开始,G可以HI,
	//剩余ACEGI
	//3.使用双指针索引 leftIndex rightIndex 上一次路径长度 lastPathLen 记录的中间点 m.midNodeSliceStep1
	//注意点:相邻点直接相连 -- 重复轮次 每轮都是endIndex = 当前path长度-1 且 可直连的时候 退出本轮
	startNode, endNode := m.InitPath[0], m.InitPath[len(m.InitPath)-1]
	leftIndex, rightIndex := 0, 1
	leftNode, rightNode := m.InitPath[leftIndex], m.InitPath[rightIndex]
	lastPathLen := len(m.InitPath)
	canConnect := false
	//1.正向弗洛伊德
	for {
		//1.相邻点直接相连
		if rightIndex-leftIndex == 1 {
			canConnect = true
		} else {
			//2.否则通过类Bresenham像素算法判断连通性
			canConnect = m.JudgeLineObstacleNew(leftNode.X, leftNode.Y, rightNode.X, rightNode.Y)
		}
		//2.判断右是否到达终点
		if rightNode == endNode {
			//右是终点 判断连通性
			if !canConnect {
				//当前不可以相连 记录终点前1个
				m.midNodeSliceStep1 = append(m.midNodeSliceStep1, m.InitPath[rightIndex-1])
			}
			//当前已完成所有中间点的添加MidNodeSlice 重置InitPath组成新的路径点
			//然后判断这次长度跟上次是否相同 相同则退出 不同则重置数据进行新一轮减枝
			//1.先重置initPath长度
			m.InitPath = m.InitPath[:0]
			//2.设置新的完整路径
			m.InitPath = append(m.InitPath, startNode)
			m.InitPath = append(m.InitPath, m.midNodeSliceStep1...)
			m.InitPath = append(m.InitPath, endNode)
			//3.重置MidNodeSlice
			m.midNodeSliceStep1 = m.midNodeSliceStep1[:0]
			//3.比较长度
			if lastPathLen == len(m.InitPath) {
				//长度相同 退出 已最大限度减枝
				//m.ResultNodeList = m.InitPath
				break
			} else {
				//长度不同 继续 重置数据
				leftIndex, rightIndex = 0, 1
				leftNode, rightNode = m.InitPath[leftIndex], m.InitPath[rightIndex]
				lastPathLen = len(m.InitPath)
				continue
			}
		} else {
			//当前没到终点 判断连通性
			if canConnect {
				//可以相连 移动右
				rightIndex++
				rightNode = m.InitPath[rightIndex]
				continue
			} else {
				//不可以相连 左边为右的上1个 并记录当前左边
				leftIndex = rightIndex - 1
				leftNode = m.InitPath[leftIndex]
				m.midNodeSliceStep1 = append(m.midNodeSliceStep1, leftNode)
				continue
			}
		}
	}
	//2.反转路径 重置必要数据给反向弗洛伊德使用 midSlice已在上次重置
	reverserSlice(&m.InitPath)                                        //现在的InitPath是反向的
	startNode, endNode = m.InitPath[0], m.InitPath[len(m.InitPath)-1] //反向的startNode和EndNode
	leftIndex, rightIndex = 0, 1
	leftNode, rightNode = m.InitPath[leftIndex], m.InitPath[rightIndex]
	lastPathLen = len(m.InitPath)
	//3.反向弗洛伊德 代码不变
	for {
		//1.相邻点直接相连
		if rightIndex-leftIndex == 1 {
			canConnect = true
		} else {
			//2.否则通过类Bresenham像素算法判断连通性
			canConnect = m.JudgeLineObstacleNew(leftNode.X, leftNode.Y, rightNode.X, rightNode.Y)
		}
		//2.判断右是否到达终点
		if rightNode == endNode {
			//右是终点 判断连通性
			if !canConnect {
				//当前不可以相连 记录终点前1个
				m.midNodeSliceStep1 = append(m.midNodeSliceStep1, m.InitPath[rightIndex-1])
			}
			//当前已完成所有中间点的添加MidNodeSlice 重置InitPath组成新的路径点
			//然后判断这次长度跟上次是否相同 相同则退出 不同则重置数据进行新一轮减枝
			//1.先重置initPath长度
			m.InitPath = m.InitPath[:0]
			//2.设置新的完整路径
			m.InitPath = append(m.InitPath, startNode)
			m.InitPath = append(m.InitPath, m.midNodeSliceStep1...)
			m.InitPath = append(m.InitPath, endNode)
			//3.重置MidNodeSlice
			m.midNodeSliceStep1 = m.midNodeSliceStep1[:0]
			//3.比较长度
			if lastPathLen == len(m.InitPath) {
				//长度相同 退出 已最大限度减枝
				//4.反转InitPath 现在变回正向
				reverserSlice(&m.InitPath)
				break
			} else {
				//长度不同 继续 重置数据
				leftIndex, rightIndex = 0, 1
				leftNode, rightNode = m.InitPath[leftIndex], m.InitPath[rightIndex]
				lastPathLen = len(m.InitPath)
				continue
			}
		} else {
			//当前没到终点 判断连通性
			if canConnect {
				//可以相连 移动右
				rightIndex++
				rightNode = m.InitPath[rightIndex]
				continue
			} else {
				//不可以相连 左边为右的上1个 并记录当前左边
				leftIndex = rightIndex - 1
				leftNode = m.InitPath[leftIndex]
				m.midNodeSliceStep1 = append(m.midNodeSliceStep1, leftNode)
				continue
			}
		}
	}
}

// 第2步:尝试从起点开始 每隔3点进行拉直操作(初始路径要3点或以上才有意义) :目的是替换连续3点的中间点
func (m *Map) jpsSmoothPathNewStep2() {
	//0.阻断长度小于等于2 上一步是InitPath
	if len(m.InitPath) <= 2 {
		m.ResultNodeList = m.InitPath
		return
	}
	//对m.InitPath进行操作
	//step2
	//TemResultSliceStep2 []*Node       //第2步临时结果切片
	//SMNodeSlice         []*Node       //step2的SM节点切片
	//MENodeSlice         []*Node       //step2的ME节点切片
	//NowNotRepeatNodeMap map[*Node]int //step2临时存储遍历经过的节点 不包括障碍
	//1.定义当前局部3点 S Old E
	startIndex := 0
	//局部3点
	nodeS := m.InitPath[startIndex]
	nodeOld := m.InitPath[startIndex+1]
	nodeE := m.InitPath[startIndex+2]
	OriginalNodeSlice := make([]*Node, 0, 3)
	OriginalNodeSlice = append(OriginalNodeSlice, nodeS, nodeOld, nodeE)
	//新的中间点
	var newY, newM *Node = nil, nil
	//固定结束点
	EndNode := m.InitPath[len(m.InitPath)-1]
	//正向局部逻辑
	for {
		//1.按顺序生成中间像素点 S - Old :SM
		m.GetMidNotObstacleFromX1ToX2ForStep2(nodeS.X, nodeS.Y, nodeOld.X, nodeOld.Y, OriginalNodeSlice, &m.SMNodeSlice)
		//2.判断SM长度 有中间M点才继续判断 否则直接下一轮3点
		if len(m.SMNodeSlice) > 0 {
			//3.顺序遍历SM
			for i := 0; i < len(m.SMNodeSlice); i++ { //m.SMNodeSlice[i]
				//4.如果按顺序有1个点能E相连 那么马上退出并记录该M点
				if m.JudgeLineObstacleNew(m.SMNodeSlice[i].X, m.SMNodeSlice[i].Y, nodeE.X, nodeE.Y) {
					newM = m.SMNodeSlice[i]
					break
				}
			}
		}
		//5.判断newM是否为nil (有没有newM能与nodeE相连)
		if newM != nil { //有newM
			//6.先暂时设置M为替换点
			newY = newM
			//7.按顺序生成M-E的中间像素点
			m.GetMidNotObstacleFromX1ToX2ForStep2(newM.X, newM.Y, nodeE.X, nodeE.Y, OriginalNodeSlice, &m.MENodeSlice)
			//8.判断MENodeSlice长度 有长度则继续 没长度则M是新的中间点
			if len(m.MENodeSlice) > 0 {
				//9.倒序遍历ME 直到有1个点能直接相连 这个点就是最后的Y点
				for i := len(m.MENodeSlice) - 1; i >= 0; i-- {
					//10.如果nodeS - ME倒序判断有1个点能与双方S和E相连，那么马上记录新的Y点退出
					if m.JudgeLineObstacleNew(nodeS.X, nodeS.Y, m.MENodeSlice[i].X, m.MENodeSlice[i].Y) {
						//11.ME有符合点 替换Y
						newY = m.MENodeSlice[i]
						break
					}
				}
			}
		}
		//End:看newY是否为nil 是否需要替换
		if newY != nil { //newM必定不为nil 有M才有Y
			if newM == newY {
				//如果就是M点 那么直接替换
				m.InitPath[startIndex+1] = newY
			} else {
				//如果当前M和Y不同 分别判断M点和Y点到 S 和 E的距离之和 谁更小 选择谁
				if m.Distance(nodeS, newM)+m.Distance(newM, nodeE) >= m.Distance(nodeS, newY)+m.Distance(newY, nodeE) {
					//如果Y的欧几里得实际距离更小或者相等 选择
					m.InitPath[startIndex+1] = newY
				} else {
					//如果M的欧几里得实际距离更小 选择M
					m.InitPath[startIndex+1] = newM
				}
			}
		}
		//AllEnd:判断当前点是不是最后一个点
		if nodeE == EndNode {
			break
		} else {
			//然后重置
			newY, newM = nil, nil
			startIndex++
			//局部3点
			nodeS = m.InitPath[startIndex]
			nodeOld = m.InitPath[startIndex+1]
			nodeE = m.InitPath[startIndex+2]
			OriginalNodeSlice[0] = nodeS
			OriginalNodeSlice[1] = nodeOld
			OriginalNodeSlice[2] = nodeE
		}
	}
	//反转initPath
	reverserSlice(&m.InitPath)
	// 重置临时数据
	//1.定义当前局部3点 S Old E
	newY, newM = nil, nil
	startIndex = 0
	//局部3点
	nodeS = m.InitPath[startIndex]
	nodeOld = m.InitPath[startIndex+1]
	nodeE = m.InitPath[startIndex+2]
	OriginalNodeSlice[0] = nodeS
	OriginalNodeSlice[1] = nodeOld
	OriginalNodeSlice[2] = nodeE
	EndNode = m.InitPath[len(m.InitPath)-1]
	//反向局部逻辑
	for {
		//1.按顺序生成中间像素点 S - Old :SM
		m.GetMidNotObstacleFromX1ToX2ForStep2(nodeS.X, nodeS.Y, nodeOld.X, nodeOld.Y, OriginalNodeSlice, &m.SMNodeSlice)
		//2.判断SM长度 有中间M点才继续判断 否则直接下一轮3点
		if len(m.SMNodeSlice) > 0 {
			//3.顺序遍历SM
			for i := 0; i < len(m.SMNodeSlice); i++ { //m.SMNodeSlice[i]
				//4.如果按顺序有1个点能E相连 那么马上退出并记录该M点
				if m.JudgeLineObstacleNew(m.SMNodeSlice[i].X, m.SMNodeSlice[i].Y, nodeE.X, nodeE.Y) {
					newM = m.SMNodeSlice[i]
					break
				}
			}
		}
		//5.判断newM是否为nil (有没有newM能与nodeE相连)
		if newM != nil { //有newM
			//6.先暂时设置M为替换点
			newY = newM
			//7.按顺序生成M-E的中间像素点
			m.GetMidNotObstacleFromX1ToX2ForStep2(newM.X, newM.Y, nodeE.X, nodeE.Y, OriginalNodeSlice, &m.MENodeSlice)
			//8.判断MENodeSlice长度 有长度则继续 没长度则M是新的中间点
			if len(m.MENodeSlice) > 0 {
				//9.倒序遍历ME 直到有1个点能直接相连 这个点就是最后的Y点
				for i := len(m.MENodeSlice) - 1; i >= 0; i-- {
					//10.如果nodeS - ME倒序判断有1个点能与双方S和E相连，那么马上记录新的Y点退出
					if m.JudgeLineObstacleNew(nodeS.X, nodeS.Y, m.MENodeSlice[i].X, m.MENodeSlice[i].Y) {
						//11.ME有符合点 替换Y
						newY = m.MENodeSlice[i]
						break
					}
				}
			}
		}
		//End:看newY是否为nil 是否需要替换
		if newY != nil { //newM必定不为nil 有M才有Y
			if newM == newY {
				//如果就是M点 那么直接替换
				m.InitPath[startIndex+1] = newY
			} else {
				//如果当前M和Y不同 分别判断M点和Y点到 S 和 E的距离之和 谁更小 选择谁
				if m.Distance(nodeS, newM)+m.Distance(newM, nodeE) >= m.Distance(nodeS, newY)+m.Distance(newY, nodeE) {
					//如果Y的欧几里得实际距离更小或者相等 选择
					m.InitPath[startIndex+1] = newY
				} else {
					//如果M的欧几里得实际距离更小 选择M
					m.InitPath[startIndex+1] = newM
				}
			}
		}
		//AllEnd:判断当前点是不是最后一个点
		if nodeE == EndNode {
			break
		} else {
			//然后重置
			newY, newM = nil, nil
			startIndex++
			//局部3点
			nodeS = m.InitPath[startIndex]
			nodeOld = m.InitPath[startIndex+1]
			nodeE = m.InitPath[startIndex+2]
			OriginalNodeSlice[0] = nodeS
			OriginalNodeSlice[1] = nodeOld
			OriginalNodeSlice[2] = nodeE
		}
	}
	//再次反转initPath 现在是Result
	reverserSlice(&m.InitPath)
	//m.ResultNodeList = m.InitPath
}

// 第3步:单点渐进 S - O - E  连接OS OE,从最接近O的点推进  --解决第2步双边被遮挡导致无效拉直的缺陷
func (m *Map) jpsSmoothPathNewStep3() {
	//0.阻断长度小于等于2 上一步是InitPath
	if len(m.InitPath) <= 2 {
		m.ResultNodeList = m.InitPath
		return
	}
	//对m.InitPath进行操作
	//step2
	//TemResultSliceStep2 []*Node       //第2步临时结果切片
	//SMNodeSlice         []*Node       //step2的SM节点切片
	//MENodeSlice         []*Node       //step2的ME节点切片
	//NowNotRepeatNodeMap map[*Node]int //step2临时存储遍历经过的节点 不包括障碍
	//1.定义当前局部3点 S Old E
	EChangeIndex, OSIndex, OEIndex := 2, 0, 0
	//局部3点
	nodeS := m.InitPath[0]
	nodeOld := m.InitPath[1]
	nodeE := m.InitPath[2]
	OriginalNodeSlice := make([]*Node, 0, 3)
	OriginalNodeSlice = append(OriginalNodeSlice, nodeS, nodeOld, nodeE)
	//新的中间点
	var newNode1, newNode2, exchangeNodeSingle, exChangeNode1, exChangeNode2 *Node = nil, nil, nil, nil, nil
	var needExchange, needDeepOS, needDeepOE = false, true, true //需要替换 需要单向继续深入的标志
	//就用没用过的ResultNodeList
	var initPathLen = len(m.InitPath)                      //定义实时更新的新路径总长(替换点位使用-不修改原initPath)
	thisResultNodeSlice := make([]*Node, 0, initPathLen*2) //定义这次渐进后的路径集合
	//Loop
	for {
		//1.按顺序生成中间像素点 OS OE
		m.GetMidNotObstacleFromX1ToX2ForStep2(nodeOld.X, nodeOld.Y, nodeS.X, nodeS.Y, OriginalNodeSlice, &m.SMNodeSlice)
		m.GetMidNotObstacleFromX1ToX2ForStep2(nodeOld.X, nodeOld.Y, nodeE.X, nodeE.Y, OriginalNodeSlice, &m.MENodeSlice)
		//2.判断SM ME长度--注意这里是OS和OE
		if len(m.SMNodeSlice) > 0 && len(m.MENodeSlice) > 0 {
			//如果2边都有长度 同时推进 直到不可视 或者有任一一边到末尾(OS OE长度可能相等 可能同时到达末尾 ) 退出进入第二阶段
			//一阶段：双向一直平推直到不可视
			for {
				newNode1 = m.SMNodeSlice[OSIndex]
				newNode2 = m.MENodeSlice[OEIndex]
				//当前可以相连 这里必定不会有索引越界问题
				if m.JudgeLineObstacleNew(newNode1.X, newNode1.Y, newNode2.X, newNode2.Y) {
					needExchange = true
					exChangeNode1 = newNode1
					exChangeNode2 = newNode2
					if OSIndex == len(m.SMNodeSlice)-1 && OEIndex == len(m.MENodeSlice)-1 { //都是最后1个
						needDeepOS = false
						needDeepOE = false
						break
					} else if OSIndex == len(m.SMNodeSlice)-1 { //OS是最后1个
						needDeepOS = false
						OEIndex++
						continue
					} else if OEIndex == len(m.MENodeSlice)-1 { //OE是最后1个
						needDeepOE = false
						OSIndex++
						continue
					} else { //都不是最后1个
						OSIndex++
						OEIndex++
						continue
					}
				} else {
					break
				}
			}
			//中间判断 当前不可以相连 如果上一次可以相连 那么回退到相连的索引
			if needExchange && needDeepOS && needDeepOE {
				//回退策略 2边都可深入才回退 并进入二阶段
				OSIndex--
				OEIndex--
				//先推进OS 之前已经回退过了
				for {
					OSIndex++
					if m.JudgeLineObstacleNew(m.SMNodeSlice[OSIndex].X, m.SMNodeSlice[OSIndex].Y, exChangeNode2.X, exChangeNode2.Y) {
						exChangeNode1 = m.SMNodeSlice[OSIndex] //更新1点
						if OSIndex == len(m.SMNodeSlice)-1 {   //如果OS最后1个都相连 那么退出 对OE进行第3阶段(因为第1阶段是双边 第2阶段是OS)
							break
						} //OS不是最后1个可以相连 默认继续下一个
					} else { //当前OS不能相连了 回退上1个OS点 退出二阶段 对OE进行第3阶段(因为第1阶段是双边 第2阶段是OS)
						OSIndex--
						break
					}
				}
				//再推进OE 之前已经回退过了
				for {
					OEIndex++
					if m.JudgeLineObstacleNew(exChangeNode1.X, exChangeNode1.Y, m.MENodeSlice[OEIndex].X, m.MENodeSlice[OEIndex].Y) {
						exChangeNode2 = m.MENodeSlice[OEIndex] //更新2点
						if OEIndex == len(m.MENodeSlice)-1 {   //如果OE最后1个都相连
							break
						} //OE不是最后1个可以相连 默认继续下一个
					} else { //当前OE不能相连了 直接退出
						break
					}
				}
			}
			//3.重置数据等:判断是否需要替换 --替换是中间1点替换成2点
			if needExchange { //有替换点
				thisResultNodeSlice = append(thisResultNodeSlice, nodeS, exChangeNode1)
				//4.判断整个逻辑结束
				if EChangeIndex == initPathLen-1 { //E点是最后1个点 当前是替换 添加回终点和终点前1个替换点 然后退出
					thisResultNodeSlice = append(thisResultNodeSlice, exChangeNode2, m.endNode)
					break
				}
				//5.否则继续新点位逻辑 当前是替换
				EChangeIndex++
				nodeS = exChangeNode2
				nodeOld = nodeE                  //init最后1个的前1个
				nodeE = m.InitPath[EChangeIndex] //init最后1个 不会越界
			} else { //无替换点
				thisResultNodeSlice = append(thisResultNodeSlice, nodeS)
				//4.判断整个逻辑结束
				if EChangeIndex == initPathLen-1 { //E点是最后1个点 当前不是替换 添加终点的前1点 和 终点
					thisResultNodeSlice = append(thisResultNodeSlice, nodeOld, nodeE)
					break
				}
				//5.否则继续新点位 当前无替换
				EChangeIndex++
				nodeS = nodeOld
				nodeOld = nodeE
				nodeE = m.InitPath[EChangeIndex] //不会越界 最多也就是最后1个点
			}
		} else if len(m.SMNodeSlice) > 0 {
			//只有OS有中间点 OE没有 OS中间点与E点判断(OS从最后1个点开始往前遍历 也就是靠近S的点 如果成功马上替换然后退出)
			for i := len(m.SMNodeSlice) - 1; i >= 0; i-- {
				if m.JudgeLineObstacleNew(m.SMNodeSlice[i].X, m.SMNodeSlice[i].Y, nodeE.X, nodeE.Y) {
					//找到替换的中间点 退出循环
					needExchange = true
					exchangeNodeSingle = m.SMNodeSlice[i]
					break
				}
			}
			//判断是否可以替换 还是不动initPath
			thisResultNodeSlice = append(thisResultNodeSlice, nodeS) //不管可不可以替换 都要
			if needExchange {                                        //当前有替换
				//4.判断整个逻辑结束
				if EChangeIndex == initPathLen-1 { //E点是最后1个点 当前不是替换 添加终点的前1点 和 终点
					thisResultNodeSlice = append(thisResultNodeSlice, exchangeNodeSingle, nodeE)
					break
				}
				//5.否则继续新点位逻辑 当前是替换
				EChangeIndex++
				nodeS = exchangeNodeSingle
				nodeOld = nodeE                  //init最后1个的前1个
				nodeE = m.InitPath[EChangeIndex] //init最后1个 不会越界
			} else { //当前无替换
				//4.判断整个逻辑结束
				if EChangeIndex == initPathLen-1 { //E点是最后1个点 当前不是替换 添加终点的前1点 和 终点
					thisResultNodeSlice = append(thisResultNodeSlice, nodeOld, nodeE)
					break
				}
				//5.否则继续新点位逻辑 当前无替换
				EChangeIndex++
				nodeS = nodeOld
				nodeOld = nodeE                  //init最后1个的前1个
				nodeE = m.InitPath[EChangeIndex] //init最后1个 不会越界
			}
		} else if len(m.MENodeSlice) > 0 {
			//只有OE有中间点 OS没有 OE中间点与S点判断(OE从最后1个点开始往前遍历 也就是靠近E的点 如果成功马上替换然后退出)
			for i := len(m.MENodeSlice) - 1; i >= 0; i-- {
				if m.JudgeLineObstacleNew(m.MENodeSlice[i].X, m.MENodeSlice[i].Y, nodeS.X, nodeS.Y) {
					//找到替换的中间点 退出循环
					needExchange = true
					exchangeNodeSingle = m.MENodeSlice[i]
					break
				}
			}
			//判断是否可以替换 还是不动initPath
			thisResultNodeSlice = append(thisResultNodeSlice, nodeS) //不管可不可以替换 都要
			if needExchange {                                        //当前有替换
				//4.判断整个逻辑结束
				if EChangeIndex == initPathLen-1 { //E点是最后1个点 当前不是替换 添加终点的前1点 和 终点
					thisResultNodeSlice = append(thisResultNodeSlice, exchangeNodeSingle, nodeE)
					break
				}
				//5.否则继续新点位逻辑 当前是替换
				EChangeIndex++
				nodeS = exchangeNodeSingle
				nodeOld = nodeE                  //init最后1个的前1个
				nodeE = m.InitPath[EChangeIndex] //init最后1个 不会越界
			} else { //当前无替换
				//4.判断整个逻辑结束
				if EChangeIndex == initPathLen-1 { //E点是最后1个点 当前不是替换 添加终点的前1点 和 终点
					thisResultNodeSlice = append(thisResultNodeSlice, nodeOld, nodeE)
					break
				}
				//5.否则继续新点位逻辑 当前无替换
				EChangeIndex++
				nodeS = nodeOld
				nodeOld = nodeE                  //init最后1个的前1个
				nodeE = m.InitPath[EChangeIndex] //init最后1个 不会越界
			}
		} else {
			//OS和OE都没有中间点
			thisResultNodeSlice = append(thisResultNodeSlice, nodeS)
			//4.判断整个逻辑结束
			if EChangeIndex == initPathLen-1 { //E点是最后1个点 当前不是替换 添加终点的前1点 和 终点
				thisResultNodeSlice = append(thisResultNodeSlice, nodeOld, nodeE)
				break
			}
			//5.否则继续新点位逻辑 当前无替换
			EChangeIndex++
			nodeS = nodeOld
			nodeOld = nodeE                  //init最后1个的前1个
			nodeE = m.InitPath[EChangeIndex] //init最后1个 不会越界
		}
		//6.重置当前方法其余数据
		OSIndex, OEIndex = 0, 0
		newNode1, newNode2, exchangeNodeSingle, exChangeNode1, exChangeNode2 = nil, nil, nil, nil, nil
		needExchange, needDeepOS, needDeepOE = false, true, true
		OriginalNodeSlice = OriginalNodeSlice[:0]
		OriginalNodeSlice = append(OriginalNodeSlice, nodeS, nodeOld, nodeE) //记得源点重置一下
	}
	//End:当前结果切片覆盖给initPath
	m.InitPath = thisResultNodeSlice
}

func getSmallerIntInThree(a, b, c int) int {
	//两两比较
	if a < b {
		if a < c {
			return a
		}
		return c
	}
	if b < c {
		return b
	}
	return c
}

func getSmallerInt(a, b int) int {
	if a > b {
		return b
	} else {
		return a
	}
}

func getBiggerInt(a, b int) int {
	if a > b {
		return a
	} else {
		return b
	}
}

// 获取int相减的绝对值
func getDisABSInt(a, b int) int {
	if a > b {
		return a - b
	} else {
		return b - a
	}
}

// 判断三个节点是否共线（通过向量叉积）
func isCollinear(a, b, c *Node) bool {
	// 计算向量AB和AC的叉积（ABx*ACy - ABy*ACx）
	abX := b.X - a.X
	abY := b.Y - a.Y
	acX := c.X - a.X
	acY := c.Y - a.Y
	return abX*acY == abY*acX
}

func reverserSlice(slice *[]*Node) {
	for i, j := 0, len(*slice)-1; i < j; i, j = i+1, j-1 {
		(*slice)[i], (*slice)[j] = (*slice)[j], (*slice)[i]
	}
}

func (m *Map) Distance(node0, node1 *Node) float64 {
	dx := math.Abs(float64(node0.X - node1.X))
	dy := math.Abs(float64(node0.Y - node1.Y))
	return math.Sqrt(dx*dx + dy*dy)
}

func (m *Map) DynamicRayCastPathFind(startX, startY, endX, endY int, slowSlice *[]*Node, average *[]int64) (bool, []int, []int) {
	// 检查起点和终点是否在地图范围内
	if !m.isMapNode(startX, startY) || !m.isMapNode(endX, endY) || m.Nodes[startX][startY].IsWall() || m.Nodes[endX][endY].IsWall() {
		fmt.Println("起点和终点不在地图范围内")
		return false, nil, nil
	}
	startTime := time.Now().UnixMicro()
	//defer func() {
	//	m.PathFindDuration = time.Now().UnixMicro() - startTime
	//}()
	//每次先不经过A星 先用Bresenham直线障碍判断算法判断当前起点终点是否可以直接相连
	firstFound := m.JudgeLineObstacleNew(startX, startY, endX, endY)
	if firstFound {
		//fmt.Println("起点能直接与终点相连")
		//fmt.Println("寻路+重置地图 耗时:", m.PathFindDuration, "μs(微秒)")
		pathX, pathY := make([]int, 0, len(m.ResultNodeList)), make([]int, 0, len(m.ResultNodeList))
		pathX = append(pathX, startX)
		pathY = append(pathY, startY)
		pathX = append(pathX, endX)
		pathY = append(pathY, endY)
		m.ResetJpsBitMap()
		return true, pathX, pathY
	}
	//startTime = time.Now().UnixMicro()
	//
	found := m.dynamicRayCastPathFind(startX, startY, endX, endY)
	if found {
		fmt.Println("初始寻路耗时:", time.Now().UnixMicro()-startTime, "μs(微秒)")
		//m.PrintOutMap() //打印已访问点地图
		//m.PrintInitNode() //打印初始路径
		//m.PrintInitPathMap() //打印初始路径地图
		//打印双向弗洛伊德后的地图
		//m.PrintInitPathMapAfterStep1Map()
		for i := 0; i < 1; i++ {
			//双向弗洛伊德
			//m.jpsSmoothPathNewStep1()
			//双向局部渐进
			m.jpsSmoothPathNewStep2()
		}
		//打印结果地图
		m.ResultNodeList = m.InitPath
		//m.printResultMap()
		//打印最终结果
		//m.printResult()
		//useTime := time.Now().UnixMicro() - startTime
		pathX, pathY := make([]int, 0, len(m.ResultNodeList)), make([]int, 0, len(m.ResultNodeList))
		for _, node := range m.ResultNodeList {
			pathX = append(pathX, node.X)
			pathY = append(pathY, node.Y)
		}
		fmt.Println("总寻路耗时:", time.Now().UnixMicro()-startTime, "μs(微秒)")
		fmt.Println("popTimes:", m.popTimes, "  pushTimes:", m.pushTimes, "  fixTimes:", m.FixTimes)
		//if useTime > 3000 {
		//	*slowSlice = append(*slowSlice, m.Nodes[endX][endY])
		//}
		//*average = append(*average, useTime)
		startTime = time.Now().UnixMicro()
		m.ResetDynamicRayCastMap()
		fmt.Println("重置地图耗时:", time.Now().UnixMicro()-startTime, "μs(微秒)")
		return true, pathX, pathY
	} else {
		fmt.Println("未找到路径!!! ", "当前起点 endX:", startX, " endY:", startY, "当前终点 endX:", endX, " endY:", endY)
		m.ResetDynamicRayCastMap()
		return false, nil, nil
	}
	//
}

func (m *Map) JpsBitPathFind(startX, startY, endX, endY int, slowSlice *[]*Node, average *[]int64) (bool, []int, []int) {
	// 检查起点和终点是否在地图范围内
	if !m.isMapNode(startX, startY) || !m.isMapNode(endX, endY) || m.Nodes[startX][startY].IsWall() || m.Nodes[endX][endY].IsWall() {
		fmt.Println("起点和终点不在地图范围内")
		return false, nil, nil
	}
	startTime := time.Now().UnixMicro()
	//defer func() {
	//	m.PathFindDuration = time.Now().UnixMicro() - startTime
	//}()
	//每次先不经过A星 先用Bresenham直线障碍判断算法判断当前起点终点是否可以直接相连
	firstFound := m.JudgeLineObstacleNew(startX, startY, endX, endY)
	if firstFound {
		//fmt.Println("起点能直接与终点相连")
		//fmt.Println("寻路+重置地图 耗时:", m.PathFindDuration, "μs(微秒)")
		pathX, pathY := make([]int, 0, len(m.ResultNodeList)), make([]int, 0, len(m.ResultNodeList))
		pathX = append(pathX, startX)
		pathY = append(pathY, startY)
		pathX = append(pathX, endX)
		pathY = append(pathY, endY)
		m.ResetJpsBitMap()
		return true, pathX, pathY
	}
	//startTime = time.Now().UnixMicro()
	//
	found := m.jpsBitPathFind(startX, startY, endX, endY)
	if found {
		fmt.Println("初始寻路耗时:", time.Now().UnixMicro()-startTime, "μs(微秒)")
		//m.PrintOutMap() //打印已访问点地图
		//m.PrintInitNode() //打印初始路径
		m.PrintInitPathMap() //打印初始路径地图
		//打印双向弗洛伊德后的地图
		//m.PrintInitPathMapAfterStep1Map()
		//for i := 0; i < 3; i++ {
		//	//双向弗洛伊德
		//	m.jpsSmoothPathNewStep1()
		//	//双向局部渐进
		//	m.jpsSmoothPathNewStep2()
		//}
		//打印结果地图
		m.ResultNodeList = m.InitPath
		//m.printResultMap()
		//打印最终结果
		//m.printResult()
		//useTime := time.Now().UnixMicro() - startTime
		pathX, pathY := make([]int, 0, len(m.ResultNodeList)), make([]int, 0, len(m.ResultNodeList))
		for _, node := range m.ResultNodeList {
			pathX = append(pathX, node.X)
			pathY = append(pathY, node.Y)
		}
		fmt.Println("总寻路耗时:", time.Now().UnixMicro()-startTime, "μs(微秒)")
		fmt.Println("popTimes:", m.popTimes, "  pushTimes:", m.pushTimes, "  fixTimes:", m.FixTimes)
		//if useTime > 3000 {
		//	*slowSlice = append(*slowSlice, m.Nodes[endX][endY])
		//}
		//*average = append(*average, useTime)
		startTime = time.Now().UnixMicro()
		m.ResetJpsBitMap()
		fmt.Println("重置地图耗时:", time.Now().UnixMicro()-startTime, "μs(微秒)")
		return true, pathX, pathY
	} else {
		fmt.Println("未找到路径!!! ", "当前起点 endX:", startX, " endY:", startY, "当前终点 endX:", endX, " endY:", endY)
		m.ResetJpsBitMap()
		return false, nil, nil
	}
	//
}

func (m *Map) APathFind(startX, startY, endX, endY int, slowSlice *[]*Node, average *[]int64) (bool, []int, []int) {
	// 检查起点和终点是否在地图范围内
	if !m.isMapNode(startX, startY) || !m.isMapNode(endX, endY) || m.Nodes[startX][startY].IsWall() || m.Nodes[endX][endY].IsWall() {
		fmt.Println("起点和终点不在地图范围内")
		return false, nil, nil
	}
	startTime := time.Now().UnixMicro()
	//defer func() {
	//	m.PathFindDuration = time.Now().UnixMicro() - startTime
	//}()
	//每次先不经过A星 先用Bresenham直线障碍判断算法判断当前起点终点是否可以直接相连
	firstFound := m.JudgeLineObstacleNew(startX, startY, endX, endY)
	if firstFound {
		fmt.Println("起点能直接与终点相连")
		//fmt.Println("寻路+重置地图 耗时:", m.PathFindDuration, "μs(微秒)")
		pathX, pathY := make([]int, 0, len(m.ResultNodeList)), make([]int, 0, len(m.ResultNodeList))
		pathX = append(pathX, startX)
		pathY = append(pathY, startY)
		pathX = append(pathX, endX)
		pathY = append(pathY, endY)
		m.ResetAstarMap()
		return true, pathX, pathY
	}
	//startTime = time.Now().UnixMicro()
	//
	found := m.aPathFind(startX, startY, endX, endY)
	if found {
		fmt.Println("初始寻路耗时:", time.Now().UnixMicro()-startTime, "μs(微秒)")
		fmt.Println("popTimes:", m.popTimes, "  pushTimes:", m.pushTimes)
		m.PrintOutMap() //打印已访问点地图
		//m.PrintInitNode()    //打印初始路径
		m.PrintInitPathMap() //打印初始路径地图
		//打印双向弗洛伊德后的地图
		//m.PrintInitPathMapAfterStep1Map()
		//双向弗洛伊德
		//m.jpsSmoothPathNewStep1()
		//双向局部渐进
		//m.jpsSmoothPathNewStep2()
		//m.jpsSmoothPathNewStep1()
		//m.jpsSmoothPathNewStep2()
		////双向局部渐进
		//m.jpsSmoothPathNewStep2()
		////双向弗洛伊德
		//m.jpsSmoothPathNewStep1()
		////双向局部渐进
		//m.jpsSmoothPathNewStep2()
		//打印结果地图
		m.ResultNodeList = m.InitPath
		m.printResultMap()
		//打印最终结果
		fmt.Println("总寻路耗时:", time.Now().UnixMicro()-startTime, "μs(微秒)")
		//fmt.Println("popTimes:", m.popTimes, "  pushTimes:", m.pushTimes)
		//m.printResult()
		//useTime := time.Now().UnixMicro() - startTime
		pathX, pathY := make([]int, 0, len(m.ResultNodeList)), make([]int, 0, len(m.ResultNodeList))
		for _, node := range m.ResultNodeList {
			pathX = append(pathX, node.X)
			pathY = append(pathY, node.Y)
		}
		//if useTime > 3000 {
		//	*slowSlice = append(*slowSlice, m.Nodes[endX][endY])
		//}
		//*average = append(*average, useTime)
		//fmt.Println("重置地图耗时:", time.Now().UnixMicro()-startTime, "μs(微秒)")
		m.ResetAstarMap()
		return true, pathX, pathY
	} else {
		fmt.Println("未找到路径!!! ", "endX:", endX, " endY:", endY)
		m.ResetAstarMap()
		return false, nil, nil
	}
	//
}

func (m *Map) ImpBTAPathFind(startX, startY, endX, endY int, slowSlice *[]*Node, average *[]int64) (bool, []int, []int) {
	// 检查起点和终点是否在地图范围内
	if !m.isMapNode(startX, startY) || !m.isMapNode(endX, endY) || m.Nodes[startX][startY].IsWall() || m.Nodes[endX][endY].IsWall() {
		fmt.Println("起点和终点不在地图范围内")
		return false, nil, nil
	}
	startTime := time.Now().UnixMicro()
	//defer func() {
	//	m.PathFindDuration = time.Now().UnixMicro() - startTime
	//}()
	//每次先不经过A星 先用Bresenham直线障碍判断算法判断当前起点终点是否可以直接相连
	firstFound := m.JudgeLineObstacleNew(startX, startY, endX, endY)
	if firstFound {
		fmt.Println("起点能直接与终点相连")
		//fmt.Println("寻路+重置地图 耗时:", m.PathFindDuration, "μs(微秒)")
		pathX, pathY := make([]int, 0, len(m.ResultNodeList)), make([]int, 0, len(m.ResultNodeList))
		pathX = append(pathX, startX)
		pathY = append(pathY, startY)
		pathX = append(pathX, endX)
		pathY = append(pathY, endY)
		m.ResetIMPBThtAMap()
		return true, pathX, pathY
	}
	//startTime = time.Now().UnixMicro()
	//
	found := m.impBTAPathFind(startX, startY, endX, endY)
	if found {
		fmt.Println("初始寻路耗时:", time.Now().UnixMicro()-startTime, "μs(微秒)")
		fmt.Println("popTimes:", m.popTimes, "  pushTimes:", m.pushTimes)
		//m.PrintOutMap() //打印已访问点地图
		//m.PrintInitNode()    //打印初始路径
		//m.PrintInitPathMap() //打印初始路径地图
		//打印双向弗洛伊德后的地图
		//m.PrintInitPathMapAfterStep1Map()
		//双向弗洛伊德
		m.jpsSmoothPathNewStep1()
		//双向局部渐进
		m.jpsSmoothPathNewStep2()
		//m.jpsSmoothPathNewStep1()
		//m.jpsSmoothPathNewStep2()
		////双向局部渐进
		//m.jpsSmoothPathNewStep2()
		////双向弗洛伊德
		//m.jpsSmoothPathNewStep1()
		////双向局部渐进
		//m.jpsSmoothPathNewStep2()
		//打印结果地图
		m.ResultNodeList = m.InitPath
		//m.printResultMap()
		//打印最终结果
		fmt.Println("总寻路耗时:", time.Now().UnixMicro()-startTime, "μs(微秒)")
		//fmt.Println("popTimes:", m.popTimes, "  pushTimes:", m.pushTimes)
		//m.printResult()
		//useTime := time.Now().UnixMicro() - startTime
		pathX, pathY := make([]int, 0, len(m.ResultNodeList)), make([]int, 0, len(m.ResultNodeList))
		for _, node := range m.ResultNodeList {
			pathX = append(pathX, node.X)
			pathY = append(pathY, node.Y)
		}
		//if useTime > 3000 {
		//	*slowSlice = append(*slowSlice, m.Nodes[endX][endY])
		//}
		//*average = append(*average, useTime)
		//fmt.Println("重置地图耗时:", time.Now().UnixMicro()-startTime, "μs(微秒)")
		m.ResetIMPBThtAMap()
		return true, pathX, pathY
	} else {
		fmt.Println("未找到路径!!! ", "endX:", endX, " endY:", endY)
		m.ResetIMPBThtAMap()
		return false, nil, nil
	}
	//
}

func (m *Map) DijieActiveBresenhamPathFind(startX, startY, endX, endY int, slowSlice *[]*Node, average *[]int64) (bool, []int, []int) {
	// 检查起点和终点是否在地图范围内
	if !m.isMapNode(startX, startY) || !m.isMapNode(endX, endY) || m.Nodes[startX][startY].IsWall() || m.Nodes[endX][endY].IsWall() {
		fmt.Println("起点和终点不在地图范围内")
		return false, nil, nil
	}
	startTime := time.Now().UnixMicro()
	//defer func() {
	//	m.PathFindDuration = time.Now().UnixMicro() - startTime
	//}()
	//每次先不经过A星 先用Bresenham直线障碍判断算法判断当前起点终点是否可以直接相连
	firstFound := m.JudgeLineObstacleNew(startX, startY, endX, endY)
	if firstFound {
		fmt.Println("起点能直接与终点相连")
		//fmt.Println("寻路+重置地图 耗时:", m.PathFindDuration, "μs(微秒)")
		pathX, pathY := make([]int, 0, len(m.ResultNodeList)), make([]int, 0, len(m.ResultNodeList))
		pathX = append(pathX, startX)
		pathY = append(pathY, startY)
		pathX = append(pathX, endX)
		pathY = append(pathY, endY)
		m.ResetIMPBThtAMap()
		return true, pathX, pathY
	}
	//startTime = time.Now().UnixMicro()
	//
	found := m.dijieActiveBresenhamPathFind(startX, startY, endX, endY)
	if found {
		fmt.Println("初始寻路耗时:", time.Now().UnixMicro()-startTime, "μs(微秒)")
		fmt.Println("popTimes:", m.popTimes, "  pushTimes:", m.pushTimes)
		//m.PrintOutMap() //打印已访问点地图
		//m.PrintInitNode()    //打印初始路径
		//m.PrintInitPathMap() //打印初始路径地图
		//打印双向弗洛伊德后的地图
		//m.PrintInitPathMapAfterStep1Map()
		//for i := 0; i < 5; i++ {
		//	//双向弗洛伊德
		//	m.jpsSmoothPathNewStep1()
		//	//双向局部渐进
		//	m.jpsSmoothPathNewStep2()
		//}
		//m.jpsSmoothPathNewStep1()
		//m.jpsSmoothPathNewStep2()
		////双向局部渐进
		//m.jpsSmoothPathNewStep2()
		////双向弗洛伊德
		//m.jpsSmoothPathNewStep1()
		////双向局部渐进
		//m.jpsSmoothPathNewStep2()
		//打印结果地图
		m.ResultNodeList = m.InitPath
		//m.printResultMap()
		//打印最终结果
		fmt.Println("总寻路耗时:", time.Now().UnixMicro()-startTime, "μs(微秒)")
		//fmt.Println("popTimes:", m.popTimes, "  pushTimes:", m.pushTimes)
		//m.printResult()
		//useTime := time.Now().UnixMicro() - startTime
		pathX, pathY := make([]int, 0, len(m.ResultNodeList)), make([]int, 0, len(m.ResultNodeList))
		for _, node := range m.ResultNodeList {
			pathX = append(pathX, node.X)
			pathY = append(pathY, node.Y)
		}
		//if useTime > 3000 {
		//	*slowSlice = append(*slowSlice, m.Nodes[endX][endY])
		//}
		//*average = append(*average, useTime)
		//fmt.Println("重置地图耗时:", time.Now().UnixMicro()-startTime, "μs(微秒)")
		m.ResetAstarMap()
		return true, pathX, pathY
	} else {
		fmt.Println("未找到路径!!! ", "endX:", endX, " endY:", endY)
		m.ResetAstarMap()
		return false, nil, nil
	}
	//
}

func (m *Map) DijiePassiveBresenhamPathFind(startX, startY, endX, endY int, slowSlice *[]*Node, average *[]int64) (bool, []int, []int) {
	// 检查起点和终点是否在地图范围内
	if !m.isMapNode(startX, startY) || !m.isMapNode(endX, endY) || m.Nodes[startX][startY].IsWall() || m.Nodes[endX][endY].IsWall() {
		fmt.Println("起点和终点不在地图范围内")
		return false, nil, nil
	}
	startTime := time.Now().UnixMicro()
	//defer func() {
	//	m.PathFindDuration = time.Now().UnixMicro() - startTime
	//}()
	//每次先不经过A星 先用Bresenham直线障碍判断算法判断当前起点终点是否可以直接相连
	firstFound := m.JudgeLineObstacleNew(startX, startY, endX, endY)
	if firstFound {
		fmt.Println("起点能直接与终点相连")
		//fmt.Println("寻路+重置地图 耗时:", m.PathFindDuration, "μs(微秒)")
		pathX, pathY := make([]int, 0, len(m.ResultNodeList)), make([]int, 0, len(m.ResultNodeList))
		pathX = append(pathX, startX)
		pathY = append(pathY, startY)
		pathX = append(pathX, endX)
		pathY = append(pathY, endY)
		m.ResetIMPBThtAMap()
		return true, pathX, pathY
	}
	//startTime = time.Now().UnixMicro()
	//
	found := m.dijiePassiveBresenhamPathFind(startX, startY, endX, endY)
	if found {
		fmt.Println("初始寻路耗时:", time.Now().UnixMicro()-startTime, "μs(微秒)")
		fmt.Println("popTimes:", m.popTimes, "  pushTimes:", m.pushTimes)
		//m.PrintOutMap() //打印已访问点地图
		//m.PrintInitNode()    //打印初始路径
		//m.PrintInitPathMap() //打印初始路径地图
		//打印双向弗洛伊德后的地图
		//m.PrintInitPathMapAfterStep1Map()
		for i := 0; i < 2; i++ {
			//双向弗洛伊德
			m.jpsSmoothPathNewStep1()
			//单点局部渐进
			m.jpsSmoothPathNewStep3()
		}
		for i := 0; i < 1; i++ {
			//双向弗洛伊德
			m.jpsSmoothPathNewStep1()
			//双向局部渐进
			m.jpsSmoothPathNewStep2()
		}
		m.jpsSmoothPathNewStep1()
		//打印结果地图
		m.ResultNodeList = m.InitPath
		m.printResultMap()
		//打印最终结果
		fmt.Println("总寻路耗时:", time.Now().UnixMicro()-startTime, "μs(微秒)")
		fmt.Println("popTimes:", m.popTimes, "  pushTimes:", m.pushTimes)
		//m.printResult()
		//useTime := time.Now().UnixMicro() - startTime
		pathX, pathY := make([]int, 0, len(m.ResultNodeList)), make([]int, 0, len(m.ResultNodeList))
		for _, node := range m.ResultNodeList {
			pathX = append(pathX, node.X)
			pathY = append(pathY, node.Y)
		}
		//if useTime > 3000 {
		//	*slowSlice = append(*slowSlice, m.Nodes[endX][endY])
		//}
		//*average = append(*average, useTime)
		//fmt.Println("重置地图耗时:", time.Now().UnixMicro()-startTime, "μs(微秒)")
		m.ResetAstarMap()
		return true, pathX, pathY
	} else {
		fmt.Println("未找到路径!!! ", "endX:", endX, " endY:", endY)
		m.ResetAstarMap()
		return false, nil, nil
	}
	//
}

// [0,1,2,3,4,5] 2 -> [0,1] [3,4,5] -> [0,1,3,4,5]
func removeAtIndex(slice *[]int, removeIndex int) {
	*slice = append((*slice)[:removeIndex], (*slice)[removeIndex+1:]...)
}
