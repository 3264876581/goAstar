package DataMgr

//	2
//
// 1 3
// 下沉：参数： arr数组 n长度 当前i索引
func down(arr *[]*Node, n int, i int) bool {
	//1.假设当前索引是最小值
	smallest := i
	for {
		//2.定义左右子节点索引
		left := 2*i + 1
		right := left + 1
		//3.判断左子节点是否存在(短路)，当前左子节点是否小于smallest
		if left >= n || left < 0 {
			break
		}
		if (*arr)[left].f < (*arr)[smallest].f {
			//更新最小索引
			smallest = left
		}
		//4.判断右子节点是否存在(短路)，当前右子节点是否小于smallest
		if right < n && (*arr)[right].f < (*arr)[smallest].f {
			//更新最小索引
			smallest = right
		}
		//5.判断现在的smallest是否等于当前索引，等于代表没有发送交换，如果不等于，则交换
		if i == smallest {
			//heapify(arr, n, smallest)
			break
		}
		//交换
		Swap(arr, i, smallest)
		//本堆完成了，但是交换过后，某个子节点可能也是其他的父节点，所以把交换后的子节点索引重新遍历heapify
		i = smallest
	}
	//如果原先的i索引已经不等于smallest，说明发生了交换，也就是下沉了，返回true
	return i != smallest
}

// 前提：原数据结构已经有堆性质
// 只需要查看当前节点的父
func up(arr *[]*Node, son int) {
	for {
		//1.计算父节点索引,不需要考虑越界，因为肯定比子节点小或相等
		parentIndex := (son - 1) / 2
		//2.son = 0,parentIndex = 0,不需要上浮交换判断，因为只有son这个元素
		//2.son = 1,parentIndex = 0,需要上浮交换判断
		//2.son = 2,parentIndex = 0,需要上浮交换判断
		if parentIndex == son || (*arr)[son].f >= (*arr)[parentIndex].f {
			break
		}
		//3.否则交换当前父子节点
		Swap(arr, son, parentIndex)
		//4.继续遍历交换后的位置，下一次计算新的父节点进行可能性上浮
		son = parentIndex
	}
}

// BuildSmallestHeap 构建小根堆|维护堆性质 (不是排序)
func BuildSmallestHeap(arr *[]*Node) {
	//1.从最后一个父节点((长度-1-1)/2)开始向上遍历构建部分堆，最终组成小根堆
	for i := (len(*arr) - 1 - 1) / 2; i >= 0; i-- {
		//2.调用down方法，构建部分堆
		down(arr, len(*arr), i)
	}
}
// SortSmallestHeap 排序小根堆 override slice from large to small
func SortSmallestHeap(arr *[]*Node) {
	//1.从最后一个父节点((长度-1-1)/2)开始向上遍历构建部分堆，最终组成小根堆
	for i := (len(*arr) - 1 - 1) / 2; i >= 0; i-- {
		//2.调用down方法，构建部分堆
		down(arr, len(*arr), i)
	}
	//2.交换首尾元素，并且遍历下沉根元素,循环从最后一个索引开始，方便交换
	for i := len(*arr) - 1; i >= 0; i-- {
		//3.交换首尾元素
		Swap(arr, 0, i)
		//4.下沉根元素，长度是不断缩小的i
		down(arr, i, 0)
	}
}

// Pop 记得外部判断长度是否大于0，大于才pop,否则报错
func Pop(arr *[]*Node) *Node {
	//赋值弹出的元素
	popEle := (*arr)[0]
	//1.判断数组长度小于等于2，外部已经判断过是否大于0了
	if len(*arr) <= 2 {
		//3.删除尾部元素
		*arr = (*arr)[:len(*arr)-1]
		//2.如果数组长度为1，直接返回第一个元素
		return popEle
	}
	//2.赋值头部元素给要弹出的元素，交换首尾元素
	Swap(arr, 0, len(*arr)-1)
	//3.删除尾部元素
	*arr = (*arr)[:len(*arr)-1]
	//4.下沉根节点元素
	down(arr, len(*arr), 0)
	//5.返回要弹出的元素
	return popEle
}

// Push (堆性质前提)添加一个元素到尾部，对该元素索引开始上浮操作
func Push(arr *[]*Node, element *Node) {
	//1.添加元素到尾部
	*arr = append(*arr, element)
	//2.上浮操作
	up(arr, len(*arr)-1)
}

// Fix 修复元素位置,原理：元素被修改后，要么比原来大，要么比原来小，如果比原来大，则下沉，如果比原来小，则上浮
func Fix(arr *[]*Node, index int) {
	//如果检测到没有下沉(一旦下沉发生交换,必定返回true,不管下沉几次)，则上浮
	if !down(arr, len(*arr), index) {
		up(arr, index)
	}
}

// Swap 交换数组元素方法
func Swap(arr *[]*Node, i, j int) {
	(*arr)[i], (*arr)[j] = (*arr)[j], (*arr)[i]
}
