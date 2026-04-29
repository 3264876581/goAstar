package main

import (
	Cus "TestServerForJps/Mgr"
	"flag"
	"fmt"
	"io"
	"log"
	"net/http"
	"os"

	"github.com/gorilla/websocket"
	jsoniter "github.com/json-iterator/go"
)

var upgrader = websocket.Upgrader{
	ReadBufferSize:  1024,
	WriteBufferSize: 1024,
} // use default options

var json = jsoniter.ConfigCompatibleWithStandardLibrary

type TestMsg struct {
	conn   *websocket.Conn
	StartX int         `json:"StartX"`
	StartY int         `json:"StartY"`
	EndX   int         `json:"EndX"`
	EndY   int         `json:"EndY"`
	Path   []NodeIndex `json:"Path"`
}
type NodeIndex struct {
	X int
	Z int
}

func newTestMsg(conn *websocket.Conn) *TestMsg {
	return &TestMsg{
		conn:   conn,
		StartX: 0,
		StartY: 0,
		EndX:   0,
		EndY:   0,
		Path:   make([]NodeIndex, 0),
	}
}

func test(w http.ResponseWriter, r *http.Request) {
	conn, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
		log.Print("upgrade:", err)
		return
	}
	defer conn.Close()
	//1.TestMsg实例化
	testMsg := newTestMsg(conn)
	//
	println("进来了")
	//2.
	for {
		_, message, err := conn.ReadMessage()
		if err != nil {
			log.Println("read:", err)
			break
		}
		//1.JSON反序列化
		err = json.Unmarshal(message, &testMsg)
		if err != nil {
			log.Println("json反序列化失败:", err)
			return
		}
		//2.进入任务
		testMsg.findPath(testMsg.StartX, testMsg.StartY, testMsg.EndX, testMsg.EndY)
	}
}

func (testMsg TestMsg) findPath(startX, startY, endX, endY int) {
	//1.接受内部
	wayPath := make([]NodeIndex, 0, 20)
	//2.寻路
	if found, pathX, pathY := m.DynamicRayCastPathFind(startX, startY, endX, endY, nil, nil); found {
		for i := 0; i < len(pathX); i++ {
			wayPath = append(wayPath, NodeIndex{
				X: pathX[i],
				Z: pathY[i],
			})
		}
		//发送给客户端
		testMsg.Path = wayPath
		//1.序列化成json字符串
		bytes, err := json.Marshal(&testMsg)
		if err != nil {
			fmt.Println("序列化json字符串失败")
			return
		}
		err = testMsg.conn.WriteMessage(1, bytes)
		if err != nil {
			fmt.Println("发送信息失败:", err)
			return
		}
	}
	//
}

var m *Cus.Map

var addr = flag.String("addr", "localhost:6733", "http service address")

func init() {
	m = Cus.NewDynamicRayCastMap(400, 400)
	loadObstacleJsonIndex(m)
}

type obstacleJsonIndex struct {
	I int `json:"i"`
	J int `json:"j"`
}

var obstacleJsonObj = make([]obstacleJsonIndex, 0, 250000)

var myMapObstacle = make([]int, 0, 250000)

func loadObstacleJsonIndex(m *Cus.Map) {
	fmt.Println("地图实例化中，正在设置障碍请等待..")
	//1.打开文件流 只读
	file, err := os.Open("./Map/mapObstacle_400.json") //只读模式打开
	if err != nil {
		fmt.Println("打开读取文件流错误")
		return
	}
	//2.读取文件流 输出byte字节数组
	bytes, err := io.ReadAll(file)
	if err != nil {
		fmt.Println("读取文件流 输出byte字节数组 错误")
		return
	}
	//3.通过json反序列化成json对象 先创建json对象
	err = json.Unmarshal(bytes, &obstacleJsonObj)
	if err != nil {
		fmt.Println("反序列化成json对象 错误")
		return
	}
	//num := ""
	//i := 0
	//for {
	//	_, err := fmt.Fscan(file, &num)
	//	if err != nil {
	//		break
	//	}
	//	for j, char := range num {
	//		if char == '1' {
	//			m.SetWall(i, j)
	//		}
	//
	//	}
	//	i++
	//	//myMapObstacle = append(myMapObstacle, num)
	//}

	//times := 0
	//for i := 0; i < 500; i++ {
	//	for j := 0; j < 500; j++ {
	//		if myMapObstacle[times] == 1 {
	//			m.SetWall(i, j)
	//		}
	//		times++
	//	}
	//}
	//fmt.Println(len(myMapObstacle))

	//4.遍历json对象切片 设置障碍
	for k := 0; k < len(obstacleJsonObj); k++ {
		m.SetWall_DynamicRayCast(obstacleJsonObj[k].I, obstacleJsonObj[k].J)
	}
	//m.CalJpsBitMap_AfterSetWall()
	m.PrintMap() //打印地图
	//5.打印
	fmt.Println("设置障碍完毕")
	//m.CalObstacleSAT()
	////fmt.Println(m.CalRectObstacleNums_UseSAT(1, 0, 4, 3))
	//fmt.Println("=========================")
	//m.CalObstacleMax_8dir()
}

func main() {
	flag.Parse()
	log.SetFlags(0)
	http.HandleFunc("/test", test)
	log.Fatal(http.ListenAndServe(*addr, nil))
}
