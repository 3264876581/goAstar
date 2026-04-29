package Cus

// 下沉 参数：操作切片指针，操作的父索引，操作的长度范围(因为pop的时候 down的操作不会影响)
func down(arr *[]*Node, downIndex, operationLen int) bool {
	//1.记录当前要操作的索引 假设为最小的索引
	fatherIndex := downIndex
	//2.开始死循环下沉操作判断 父节点索引为n，左子为2 * n + 1
	for { //-------------------------------------------------注意以下操作全都是fatherIndex(downIndex不变留着判断返回bool)
		//-------------------------------------左子节点
		//3.计算左子节点
		sonLeftIndex := 2*fatherIndex + 1
		//4.判断左子节点范围是否越界 越界代表根本没有子节点那么退出下沉逻辑
		if sonLeftIndex >= operationLen || sonLeftIndex < 0 {
			break
		}
		//5.假设当前最小索引为左子节点
		temSmallest := sonLeftIndex
		//-------------------------------------右子节点
		//6.左子节点存在 那么先计算右子节点索引
		sonRightIndex := sonLeftIndex + 1
		//7.如果右子节点也在范围内并且右子节点值小于左子节点的值 更新当前最小为右
		if sonRightIndex < operationLen && (*arr)[sonRightIndex].F < (*arr)[sonLeftIndex].F {
			temSmallest = sonRightIndex
		}
		//-------------------------------------子比父
		//8.拿当前最小的子节点再和父节点比较 子节点小就交换 否则父节点还是最小的那个 不交换退出下沉操作
		if (*arr)[temSmallest].F < (*arr)[fatherIndex].F {
			//9.交换自身 和 arr 索引--当前参数为没有交换位置的 temSmallest 和 没有改变的父节点索引 fatherIndex
			swapValueAndUpdateInnerIndex(arr, temSmallest, fatherIndex)
			//10.更新最小父索引为交换后的子节点索引 因为这个最新的索引要被当做下一次循环的父索引
			fatherIndex = temSmallest
		} else {
			break
		}
	}
	//11.当上面的循环break到这的时候代表下沉操作全部完成 有必要返回当前是否进行了1次或更多次的下沉操作的bool(只要下沉成功就返回true)
	return downIndex != fatherIndex //downIndex是一开始进来的第1个要操作的父节点 一直没有改变,fatherIndex只要下沉了,一定会改变
}

// 上浮(用处:Push 和 Fix) 参数:切片，上浮索引(当做某个子索引,默认一定 >= 0)
func up(arr *[]*Node, upIndex int) {
	//例如：upIndex是某子节点 fatherIndex * 2 + 1 = 左子节点 || fatherIndex * 2 + 2 = 右子节点
	//则获取父节点公式为 fatherIndex = (upIndex - 1)/2
	//验证：upIndex = 0,fatherIndex = -0.5 去除小数部分 = 0
	//	   upIndex = 1,fatherIndex = 0    去除小数部分 = 0
	//	   upIndex = 2,fatherIndex = 0.5  去除小数部分 = 0
	//	   upIndex = 3,fatherIndex = 1    去除小数部分 = 1
	//	   upIndex = 4,fatherIndex = 1.5  去除小数部分 = 1
	//0.上浮成功 交换后 新的父节点也要被当做新的子节点继续上浮操作
	for {
		//1.先计算当前子节点的父节点
		fatherIndex := (upIndex - 1) / 2
		//2.判断父节点 第一种情况当前只有1个节点 和 当前父节点依然是最小的那个 那么不进行逻辑
		if upIndex == fatherIndex || (*arr)[fatherIndex].F < (*arr)[upIndex].F {
			break
		}
		//3.否则交换当前父子索引 和 存储索引
		swapValueAndUpdateInnerIndex(arr, upIndex, fatherIndex)
		//4.更新下一次用到的新的子索引 为 这次的父索引
		upIndex = fatherIndex
	}
}

// Fix 修复(更新) 基于堆性质修改某个索引值，利用down返回bool判断通过down还是up再次维护堆性质
func Fix(arr *[]*Node, fixIndex int) {
	//1.先尝试下沉成不成功 不成功则尝试上浮
	if !down(arr, fixIndex, len(*arr)) {
		up(arr, fixIndex)
	}
}

// Push 尾部添加(一定要在具有堆性质的切片上操作) ：尾部添加新节点(注意 添加到尾部后 先为当前节点更新赋值新的索引存储) 然后上浮维护堆性质
func Push(arr *[]*Node, node *Node) {
	//0.添加到尾部
	*arr = append(*arr, node)
	//1.更新节点存储在openList中的索引为最后1位
	node.IndexInOpenList = len(*arr) - 1
	//2.上浮
	up(arr, node.IndexInOpenList)
}

// Pop 头部弹出(重要 ！！！ 一定要在具有堆性质的切片上操作,外部判断切片长度大于0) : 先交换到尾部记录后删除 然后 下沉头部(因为尾部已经删除 当前下沉长度范围就是切片长度)
func Pop(arr *[]*Node) *Node {
	//0.记录要弹出的节点 实际是指针存储了地址 这个地址是另一个节点指针的内存地址 节点指针存储的才是实际的Node结构体堆内存地址
	//a[c] -> [c1,c2,c3....](切片存的就是一个个堆地址值) -> c[value] ,当前操作是node拷贝了这个地址值 2层引用
	node := (*arr)[0]
	lenArr := len(*arr) - 1
	//1.交换首尾元素 和元素存储的索引
	swapValueAndUpdateInnerIndex(arr, 0, lenArr)
	//2.删除尾部元素(切片操作 实际只是长度-1 底层数组内存占用没有改变)
	*arr = (*arr)[:lenArr]
	//3.下沉头节点 下沉范围就是当前已经减少过长度的arr
	down(arr, 0, lenArr)
	//4.把当前openIndex重置为-1
	node.IndexInOpenList = -1
	//5.返回
	return node
}

// 交换自身 和 arr 索引
func swapValueAndUpdateInnerIndex(arr *[]*Node, a, b int) {
	// 先交换节点
	(*arr)[a], (*arr)[b] = (*arr)[b], (*arr)[a]
	// 再更新索引
	(*arr)[a].IndexInOpenList = a
	(*arr)[b].IndexInOpenList = b
}

//1 3 2 4 5
//1 6 2 4 5
