
/*
版本更新信息
2013.12.19. 开始思考二维矩形装箱问题。

2014.5.17. 基本算法B0第一版程序写完，程序名为RecPackingB0

2014.5.26. 穴度枚举算法B1第一版程序写完，程序名为RecPackingB1

2014.5.29. 优美度枚举算法B2第一版程序写完，程序名为RecPackingB2

2014.9.3.  优美度枚举算法第二版程序RecPackingB2b写完，在算法和编程上相比于第一版有较大改进。

2014.9.10. 开始写优美度枚举算法第三版程序RecPackingB2c，主要是为了程序逻辑清晰，注释完善。

2014.9.13. 开始写优美度枚举算法第四版程序RecPackingB2d。
calculate_num_of_corners_increament函数中需要查找VERTEX数组，由顺序查找改为折半查找。
do_an_action函数和current_cornet_is_90_degree函数中同样需要查找VERTEX数组，由顺序查找改为折半查找。

2014.9.14. debug：应该考虑。顶点数组中的那些顶点，是否顶住本动作对应矩形块的四条边。（顶点恰好位于边的端点的，不算。）

2014.9.15. 基本算法B0第二版算法写完，名为RecPackingB0a：删去无用的数据结构和函数，增加注释，改进程序。

2014.9.15. RecPackingB0aL由名为RecPackingB0a略作修改而来。程序用于计算strip packing问题实例较方便。

2014.9.20. debug：VERTEX[index_vertex].number_of_edge应该赋初值为0。
优美度枚举算法第五版程序RecPackingB2e写完。RecPackingB2eL1在RecPackingB2e基础上略作修改。用于计算strip packing问题。

2014.9.20. 基本算法B0第三版算法写完，名为RecPackingB0b：删去无用的数据结构和函数，改进程序，去掉bug。

2014.9.20. RecPackingB0bL由名为RecPackingB0b略作修改而来。程序用于计算strip packing问题实例较方便。

本程序仅可用于教学。若用于商业用途，则须作者同意。
作者：王磊   武汉科技大学计算机科学与技术学院 软件工程系  email:wanglei77@wust.edu.cn
Copyright: Wang Lei. All rights reserved.
*/

/*
参考文献：
[1] 王磊,尹爱华. 求解二维矩形Packing问题的一种优美度枚举算法. 中国科学. 2014. 已投稿，第一轮评审完毕。
[2] 何琨,黄文奇,金燕. 基于动作空间求解二维矩形Packing问题的高效算法. 软件学报. 2012, 23(12):1037-1044
[3] Defu Zhang, Lijun Wei, Stephen C.H.Leung, Qingshan Chen. A binary search heuristic algorithm based on randomized
local search for the rectangular strip-packing problem. INFORMS Journal on Computing. 2013, 25(2):332-345
*/

#include <stdio.h>		//printf函数需要包含stdio.h
#include <stdlib.h>		//malloc函数和free函数需要包含stdlib.h
#include <string.h>		//strcpy函数需要包含string.h
#include <time.h>		//clock函数需要包含time.h

//graphics.h头文件用于显示图形
#include <graphics.h>
#include <windows.h>

#include <math.h>



//--------------------------------------------------------------------------------------------------------
//结构体的定义
//--------------------------------------------------------------------------------------------------------

//定义1.格局，由结构体RectangleInfo表示。
//struct RectangleInfo：木块的相关信息。要说清楚木块是否在盒内。若在盒内，其姿态和坐标。
struct RectangleInfo
{
	int index;	//木块的序号。同时也是此木块在木块宽度和木块高度数组中的下标
	int	flag;	//0表示在盒外。1表示在盒内。
	int width;	//木块宽度  长边长
	int height;	//木块高度  短边长
	int x;		//木块左下角x坐标
	int y;		//木块左下角y坐标
	int pose;	//0表示躺着。1表示站着。
};

//定义2.动作
struct Action
{
	int	index;	//木块的序号。同时也是此木块在木块宽度和木块高度数组中的下标		哪个木块？
	int x;		//木块左下角x坐标													什么位置？
	int y;		//木块左下角y坐标
	int pose;	//0表示躺着。1表示站着。											何种姿态？
	int index_actionspace;					//动作在哪个动作空间中？
	
	int width;	//木块宽度  长边长
	int height;	//木块高度  短边长

	int num_of_lines;						//动作与动作空间的贴边数
	int num_of_corners_increament;			//动作做完以后，剩余空间的角区数的增加值。
	int beauty_degree;						//动作的优美度
};

//定义3.动作空间
struct ActionSpace
{
	int x;		//动作空间左下角x坐标
	int y;		//动作空间左下角y坐标
	int width;	//动作空间宽度  x方向延伸
	int height;	//动作空间高度  y方向延伸

	int flag;	//1表示有效，0表示无效。

	int corner_90_degree[4];	//动作空间左下、右下、右上、左上（用下标0，1，2，3表示）四个角是否为90度角区。取值：0非，1是
};

//定义4.木块的角
struct Angle
{
	int index;	//角属于哪个木块，木块在木块宽度和木块高度数组中的下标。下标-1表示特殊的木块，即矩形框。
	int type;	//角的类型：0-3。分别表示左下、右下、右上、左上。
};

//定义5.木块的边
struct Edge
{
	int index;	//边属于哪个木块，木块在木块宽度和木块高度数组中的下标。下标-1表示特殊的木块，即矩形框。
	int type;	//边的类型：0-3。分别表示下、右、上、左。

	int x1;		//边的两个端点的坐标，(x1,y1)和(x2,y2)
	int y1;
	int x2;
	int y2;
};

//定义6.顶点
struct Vertex
{
	int x;					//顶点坐标(x,y)
	int y;

	int number_of_angles;	//有多少个木块角正好在这个顶点。值为1至4
	int index_of_angles[4];	//角在角数组中的下标
	
	int number_of_edge;		//有多少条边经过这个顶点。边的端点是这个顶点的不算。 值为0或1
};



//----------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------


//============================================================================================================================
//============================================全局变量========================================================================
int BOX_WIDTH,BOX_HEIGHT;		//盒子的宽度和高度
int NUM_OF_RECTANGLES;			//木块数
int * WIDTHS_OF_RECTANGLES;		//全局变量WIDTHS_OF_RECTANGLES是木块宽度数组。WIDTHS_OF_RECTANGLES[i]的值为i号木块的宽度。
int * HEIGHTS_OF_RECTANGLES;	//全局变量HEIGHTS_OF_RECTANGLES是木块高度数组。HEIGHTS_OF_RECTANGLES[i]的值为i号木块的高度。

struct RectangleInfo * RECTANGLES_INFO;	//木块信息数组。表示格局。

struct ActionSpace * ACTION_SPACE;		//动作空间数组。记录当前格局中，所有动作空间的信息。
int pointer_to_ACTION_SPACE;			//pointer_to_ACTION_SPACE的值为当前格局中，动作空间个数。

struct Angle * ANGLE;					//矩形角数组。记录当前格局中，所有放入矩形框的木块的角的信息。
int pointer_to_ANGLE;					//pointer_to_ANGLE的值为当前格局中，放入矩形框的木块的角个数。

struct Edge * EDGE;						//矩形边数组。记录当前格局中，所有放入矩形框的木块的边的信息。	
int pointer_to_EDGE;					//pointer_to_EDGE的值为当前格局中，放入矩形框的木块的边数。

struct Vertex * VERTEX;					//顶点数组。记录当前格局中，所有放入矩形框的木块以及矩形框本身的顶点信息。
int pointer_to_VERTEX;					//pointer_to_VERTEX的值为当前格局中的顶点数。（重合的顶点算一个）



struct Action * ACTION_SEQ;//所做的动作序列
int pointer_to_ACTION_SEQ;

int SUM_OF_REC_AREA;	//木块总面积

//=============================================================================================================================


//===================================================================================================================
//==============================================函数说明=============================================================
void build_an_instance(FILE * fp);

void init_data();

void do_an_action(struct Action action, int index_actionspace);

void free_mem();

int current_corner_is_90_degree(struct ActionSpace current_actionspace, int corner_type);

int calculate_num_of_corners_increament(struct Action action, int corner_type);

int current_action_is_better(struct Action current_action, struct Action best_action);

int basic_algorithm_choose_an_action(struct Action * ptr_action, int * ptr_index_actionspace);

int rectangle_is_prior(int i, int j);

int half_search_vertex(struct Vertex * ptr_vertex, int number_of_vertex, int x, int y);

//===================================================================================================================

/*
	函数build_an_instance()
	参数：FILE * fp
	返回值：void 
	功能：1.调用fscanf库函数读取benchmark算例文件中的数据。
	2.调用malloc库函数分配内存。

  benchmark算例文件的格式举例说明如下：
  3   木块数
  20  盒子高度
  0  3  5      第一个木块宽度为3，高度为5。本行的三个整数分别表示木块序号、木块宽度、木块高度。注意木块从0开始编号。
  1  4  4      依此类推，第二个木块
  2  6  3      第三个木块

*/
void build_an_instance(FILE * fp)
{
	int i,temp;

	fscanf(fp, "%d", &NUM_OF_RECTANGLES);
	fscanf(fp, "%d", &BOX_HEIGHT);
	WIDTHS_OF_RECTANGLES = (int *)malloc(sizeof(int) * NUM_OF_RECTANGLES);
	HEIGHTS_OF_RECTANGLES = (int *)malloc(sizeof(int) * NUM_OF_RECTANGLES);

	for(i=0; i<NUM_OF_RECTANGLES; i++)
	{
		fscanf(fp, "%d", &temp);
		fscanf(fp, "%d", &WIDTHS_OF_RECTANGLES[i]);
		fscanf(fp, "%d", &HEIGHTS_OF_RECTANGLES[i]);
	}

	RECTANGLES_INFO = (struct RectangleInfo *)malloc(sizeof(struct RectangleInfo) * NUM_OF_RECTANGLES); 

	ACTION_SPACE = (struct ActionSpace *)malloc(sizeof(struct ActionSpace) * (10*NUM_OF_RECTANGLES+4));

	ANGLE = (struct Angle *)malloc(sizeof(struct Angle) * (4*NUM_OF_RECTANGLES+4));

	EDGE = (struct Edge *)malloc(sizeof(struct Edge) * (4*NUM_OF_RECTANGLES+4));

	VERTEX = (struct Vertex *)malloc(sizeof(struct Vertex) * (3*NUM_OF_RECTANGLES+4));

	ACTION_SEQ = (struct Action *)malloc(sizeof(struct Action) * NUM_OF_RECTANGLES);

}

/*
	函数init_data()
	参数：无
	返回值：void 
	功能：初始化数据结构

*/
void init_data()
{
	int i,j,t;

	//0. 要求木块宽度大于等于木块高度。如果木块宽度小于木块宽度，则木块宽度和高度值交换。这样做是为了计算方便。
	for(i=0; i<NUM_OF_RECTANGLES; i++)
	{
		if(WIDTHS_OF_RECTANGLES[i] < HEIGHTS_OF_RECTANGLES[i])
		{
			t = WIDTHS_OF_RECTANGLES[i];
			WIDTHS_OF_RECTANGLES[i] = HEIGHTS_OF_RECTANGLES[i];
			HEIGHTS_OF_RECTANGLES[i] = t;
		}
	}

	//1. 给木块排序。排序规则为<周长（大优），长边长（大优），短边长（大优），下标（小优）>
	//需要调用函数rectangle_is_prior。
	for(i=0; i<NUM_OF_RECTANGLES-1; i++)
	{
		for(j=0; j<NUM_OF_RECTANGLES-1-i; j++)
		{
			if(rectangle_is_prior(j, j+1) == 0)
			{
				t = WIDTHS_OF_RECTANGLES[j];
				WIDTHS_OF_RECTANGLES[j] = WIDTHS_OF_RECTANGLES[j+1];
				WIDTHS_OF_RECTANGLES[j+1] = t;

				t = HEIGHTS_OF_RECTANGLES[j];
				HEIGHTS_OF_RECTANGLES[j] = HEIGHTS_OF_RECTANGLES[j+1];
				HEIGHTS_OF_RECTANGLES[j+1] = t;
			}
		}
	}

	//2. RECTANGLES_INFO数组初始化，所有木块均在盒子外面。
	for(i=0; i<NUM_OF_RECTANGLES; i++)
	{
		RECTANGLES_INFO[i].index = i;
		RECTANGLES_INFO[i].flag = 0;
		RECTANGLES_INFO[i].width = WIDTHS_OF_RECTANGLES[i];
		RECTANGLES_INFO[i].height = HEIGHTS_OF_RECTANGLES[i];
	}

	//3. ACTION_SPACE数组初始化
	ACTION_SPACE[0].x = 0;
	ACTION_SPACE[0].y = 0;
	ACTION_SPACE[0].width = BOX_WIDTH;
	ACTION_SPACE[0].height = BOX_HEIGHT;
	ACTION_SPACE[0].flag = 1;
	pointer_to_ACTION_SPACE = 1;

	//4. ANGLE数组初始化
	ANGLE[0].index = -1;
	ANGLE[0].type = 0;
	ANGLE[1].index = -1;
	ANGLE[1].type = 1;
	ANGLE[2].index = -1;
	ANGLE[2].type = 2;
	ANGLE[3].index = -1;
	ANGLE[3].type = 3;
	pointer_to_ANGLE = 4;

	//5. EDGE数组初始化
	//边的端点坐标(x1,y1)和(x2,y2)。x1<=x2, y1<=y2。
	EDGE[0].index = -1;
	EDGE[0].type = 0;
	EDGE[0].x1 = 0;				EDGE[0].y1 = 0;
	EDGE[0].x2 = BOX_WIDTH;		EDGE[0].y2 = 0;

	EDGE[1].index = -1;
	EDGE[1].type = 1;
	EDGE[1].x1 = BOX_WIDTH;		EDGE[1].y1 = 0;
	EDGE[1].x2 = BOX_WIDTH;		EDGE[1].y2 = BOX_HEIGHT;	
	
	EDGE[2].index = -1;
	EDGE[2].type = 2;
	EDGE[2].x1 = 0;				EDGE[2].y1 = BOX_HEIGHT;
	EDGE[2].x2 = BOX_WIDTH;		EDGE[2].y2 = BOX_HEIGHT;	

	EDGE[3].index = -1;
	EDGE[3].type = 3;
	EDGE[3].x1 = 0;				EDGE[3].y1 = 0;
	EDGE[3].x2 = 0;				EDGE[3].y2 = BOX_HEIGHT;	

	pointer_to_EDGE = 4;

	//6. VERTEX数组初始化
	//VERTEX数组排序优先序：<y（小优先），x（小优先）>
	VERTEX[0].x = 0;
	VERTEX[0].y = 0;
	VERTEX[0].number_of_angles = 1;
	VERTEX[0].index_of_angles[0] = 0;
	VERTEX[0].number_of_edge=0;

	VERTEX[1].x = BOX_WIDTH;
	VERTEX[1].y = 0;
	VERTEX[1].number_of_angles = 1;
	VERTEX[1].index_of_angles[0] = 1;
	VERTEX[1].number_of_edge=0;

	VERTEX[2].x = 0;
	VERTEX[2].y = BOX_HEIGHT;
	VERTEX[2].number_of_angles = 1;
	VERTEX[2].index_of_angles[0] = 3;
	VERTEX[2].number_of_edge=0;

	VERTEX[3].x = BOX_WIDTH;
	VERTEX[3].y = BOX_HEIGHT;
	VERTEX[3].number_of_angles = 1;
	VERTEX[3].index_of_angles[0] = 2;
	VERTEX[3].number_of_edge=0;


	pointer_to_VERTEX = 4;


	//7.动作序列数组初始化
	pointer_to_ACTION_SEQ = 0;

	//8.计算待放矩形块总面积
	SUM_OF_REC_AREA = 0;
	for(i=0; i<NUM_OF_RECTANGLES; i++)
	{
		SUM_OF_REC_AREA += RECTANGLES_INFO[i].width * RECTANGLES_INFO[i].height;
	}

	
}


/*
	函数rectangle_is_prior()
	参数：int i, int j
	返回值：int 
	功能：判断下标为i的矩形是否比下标为j的矩形优先：若是，则返回1。若不是则返回0。
	被init_data函数调用。

*/
int rectangle_is_prior(int i, int j)
{
	if(WIDTHS_OF_RECTANGLES[i] + HEIGHTS_OF_RECTANGLES[i] > WIDTHS_OF_RECTANGLES[j] + HEIGHTS_OF_RECTANGLES[j])
		return 1;
	else if(WIDTHS_OF_RECTANGLES[i] + HEIGHTS_OF_RECTANGLES[i] == WIDTHS_OF_RECTANGLES[j] + HEIGHTS_OF_RECTANGLES[j])
	{
		if(WIDTHS_OF_RECTANGLES[i] > WIDTHS_OF_RECTANGLES[j])
			return 1;
		else if(WIDTHS_OF_RECTANGLES[i] == WIDTHS_OF_RECTANGLES[j])
		{
			if(HEIGHTS_OF_RECTANGLES[i] > HEIGHTS_OF_RECTANGLES[j])
				return 1;
			else if(HEIGHTS_OF_RECTANGLES[i] == HEIGHTS_OF_RECTANGLES[j])
			{
				if(i<j)
					return 1;
			}
		}
	}

	return 0;
}

/*
	函数free_mem()
	参数：无
	返回值：void 
	功能：调用free库函数释放之前用malloc库函数分配的内存。

*/
void free_mem()
{

	free(WIDTHS_OF_RECTANGLES);
	free(HEIGHTS_OF_RECTANGLES);
	
	free(RECTANGLES_INFO);

	free(ACTION_SPACE);

	free(ANGLE);

	free(EDGE);

	free(VERTEX);

	free(ACTION_SEQ);

}

/*
	函数current_corner_is_90_degree()
	参数：struct ActionSpace current_actionspace, int corner_type
	返回值：int 
	功能：判断当前考虑的动作空间的一个角是不是90度角区：若是，则返回1。若不是则返回0。

*/
int current_corner_is_90_degree(struct ActionSpace current_actionspace, int corner_type)
{
	int x,y,i,flag,index_vertex,index_of_angle1,index_of_angle2,angle1_type,angle2_type;

	//算出角的顶点的x,y坐标。分四种情况讨论。corner_type取值为0，1，2，3四种。
	if(corner_type == 0)			//corner_type值为0，表示考虑动作空间的左下角
	{
		x = current_actionspace.x;	//current_actionspace.x是动作空间左下角x坐标
		y = current_actionspace.y;	//current_actionspace.y是动作空间左下角y坐标
	}	
	else if(corner_type == 1)		//corner_type值为1，表示考虑动作空间的右下角
	{
		x = current_actionspace.x + current_actionspace.width;
		y = current_actionspace.y;
	}
	else if(corner_type == 2)		//corner_type值为2，表示考虑动作空间的右上角
	{
		x = current_actionspace.x + current_actionspace.width;
		y = current_actionspace.y + current_actionspace.height;
	}
	else							//corner_type值为3，表示考虑动作空间的左上角
	{
		x = current_actionspace.x;
		y = current_actionspace.y + current_actionspace.height;
	}

	//当前考虑的动作空间的这个角，必须是已放入矩形框的某个矩形块（包括矩形框）顶点所在处。若不是，则返回0。
	//顺序查找 beging-------------------------------------------------------------
	//flag = 0;	//flag标记初始值设为0
	//for循环查找VERTEX数组。
	//for(i=0; i<pointer_to_VERTEX; i++)
	//{
	//	if(x==VERTEX[i].x && y==VERTEX[i].y)
	//	{
	//		flag = 1;
	//		index_vertex = i;	//如果在VERTEX数组中找到了，则在VERTEX数组中的下标记在index_vertex变量中。flag标记置为1。
	//		break;
	//	}
	//}

	//if(flag == 0)	//如果for循环结束时，还没有找到，则flag值依然等于0。本函数返回0。
	//	return 0;
	//顺序查找 end-----------------------------------------------------------------

	//折半查找 beign------------------------------------------------------
	index_vertex = half_search_vertex(VERTEX, pointer_to_VERTEX, x, y);

	if(index_vertex == -1)//在顶点数组中未找到这个点
		flag = 0;
	else
		flag = 1;

	if(flag == 0)	//如果在顶点数组中未找到这个点。本函数返回0。
		return 0;
	//折半查找 end---------------------------------------------------------

	//当前考虑的动作空间的这个角，是矩形框的一个角吗？若是，则返回1。
	//如果当前考虑的动作空间的一个角正好是矩形框的角，则必然为90度角区。
	if(x==0 && y==0 || x==BOX_WIDTH && y==0 || x==BOX_WIDTH && y==BOX_HEIGHT || x==0 && y==BOX_HEIGHT)
		return 1;

	//有没有边经过这个顶点？
	//如果有某个矩形块（包括矩形框）的一条边经过这个顶点（边的端点正好在这个顶点的，不算），则这个顶点所在处肯定是90度角区。
	if(VERTEX[index_vertex].number_of_edge == 1)
		return 1;
	else	//没有某个矩形块（包括矩形框）的一条边经过这个顶点
	{
		//分四种情况讨论。根据这个点所在处有多少个矩形块的顶点重合来作讨论，分为1，2，3，4四种情形。
		if(VERTEX[index_vertex].number_of_angles == 1)	//恰有1个矩形块的顶点，肯定不是90度角区。
			return 0;
		else if(VERTEX[index_vertex].number_of_angles == 2)//恰有2个矩形块的顶点，这时需要进一步考察：如果两个矩形块相邻，
		{												   //则不是90度角区。反之，如果两个矩形块不相邻，则必然构成90度角区。
			index_of_angle1 = VERTEX[index_vertex].index_of_angles[0];
			index_of_angle2 = VERTEX[index_vertex].index_of_angles[1];

			angle1_type = ANGLE[index_of_angle1].type;
			angle2_type = ANGLE[index_of_angle2].type;

			if(angle1_type==0 && angle2_type==2 || angle1_type==1 && angle2_type==3 || angle1_type==2 && angle2_type==0 || angle1_type==3 && angle2_type==1)
				return 1;
			else
				return 0;
		}
		else if(VERTEX[index_vertex].number_of_angles == 3)	//恰有3个矩形块的顶点，肯定是90度角区。
			return 1;
		else												//恰有4个矩形块的顶点，肯定不是90度角区。
			return 0;
	}
	

}

/*
	函数half_search_vertex()
	参数：struct Vertex * ptr_vertex, int number_of_vertex，int x，int y
	返回值：int 
	功能：在顶点数组中查找一个顶点。若找不到，则返回-1；找到了则返回元素下标。
	折半查找。顶点数组中的顶点已经排序：<y（小优先），x（小优先）>

*/
int half_search_vertex(struct Vertex * ptr_vertex, int number_of_vertex, int x, int y)
{
	int head,mid,tail;

	head = 0;					//2014.9.13. debug ：head，tail未赋初值。已改正。 
	tail = number_of_vertex-1;
	while(head<=tail)
	{
		mid = (head+tail)/2;
		if(ptr_vertex[mid].x == x && ptr_vertex[mid].y == y)//已找到
			return mid;
		
		if(ptr_vertex[mid].y < y || ptr_vertex[mid].y == y && ptr_vertex[mid].x < x)
			head = mid+1;
		else
			tail = mid-1;
	}

	return -1;
}

/*
	函数calculate_num_of_corners_increament()
	参数：struct Action action, int corner_type  action占其所在动作空间的corner_type这个角。
	返回值：int 
	功能：计算出本动作action做完以后，剩余空间角区数比做本动作之前增加了多少。

*/
int calculate_num_of_corners_increament(struct Action action, int corner_type)
{
	int num_of_dismiss_corners,rec_width,rec_height,i;
	struct ActionSpace current_actionspace;
	int index_vertex,flag,index_angle,angle_type,x,y;

	//预备工作
	num_of_dismiss_corners = 1;//num_of_dismiss_corners表示消去的角区数。占角动作所占的那个角肯定消去了。

	//木块宽度一定大于等于木块高度。action.width和action.height表示木块的宽度和高度。
	//rec_width表示木块放下后，在x轴方向上的长度。rec_height表示木块放下后，在y轴方向上的长度。
	if(action.pose == 0)//木块躺着
	{
		rec_width = action.width;
		rec_height = action.height;
	}
	else//木块站着
	{
		rec_width = action.height;
		rec_height = action.width;
	}

	//current_actionspace表示本矩形块放下以后，所占据的空间。此空间正好与矩形块相等。
	current_actionspace.x = action.x;
	current_actionspace.y = action.y;
	current_actionspace.width = rec_width;
	current_actionspace.height = rec_height;
	
	//我们检查这个矩形空间，只可能在其四个角的地方出现消去角区的现象。
	for(i=0; i<=3; i++)
	{
		if(i==corner_type)//占角动作所占的那个角肯定消去了。不必再考虑
			continue;


		//x，y是正在考虑的这个矩形空间的一个角的坐标。
		if(i==0)		
		{
			x = action.x;
			y = action.y;
		}
		else if(i==1)
		{
			x = action.x + rec_width;
			y = action.y;
		}
		else if(i==2)
		{
			x = action.x + rec_width;
			y = action.y + rec_height;
		}
		else
		{
			x = action.x;
			y = action.y + rec_height;
		}

		//检查这个角是不是90度角区
		if(current_corner_is_90_degree(current_actionspace,i) == 1)
		{
			num_of_dismiss_corners ++;
		}
		else
		{
			//若此处（action动作的i这个角）正好有一个角（270度角区），与之相邻，也会消去一个角。
			//顺序查找的程序段 begin-------------------------------------------------------
			//flag = 0;
			//for(index_vertex=0; index_vertex<pointer_to_VERTEX; index_vertex++)
			//{
			//	if(VERTEX[index_vertex].x==x && VERTEX[index_vertex].y==y)
			//	{
			//		flag = 1;
			//		break;
			//	}
			//}
			//顺序查找的程序段 end----------------------------------------------------------

			//折半查找
			index_vertex = half_search_vertex(VERTEX, pointer_to_VERTEX, x, y);

			if(index_vertex == -1)//在顶点数组中未找到这个点
				continue;

			if(VERTEX[index_vertex].number_of_edge == 1  ||  VERTEX[index_vertex].number_of_angles != 1)
				continue;

			index_angle = VERTEX[index_vertex].index_of_angles[0];
			angle_type = ANGLE[index_angle].type;

			if(i == 0)
			{
				if(angle_type == 1 || angle_type == 3)
					num_of_dismiss_corners ++;
			}
			else if(i == 1)
			{
				if(angle_type == 0 || angle_type == 2)
					num_of_dismiss_corners ++;
			}
			else if(i == 2)
			{
				if(angle_type == 1 || angle_type == 3)
					num_of_dismiss_corners ++;
			}
			else if(i == 3)
			{
				if(angle_type == 0 || angle_type == 2)
					num_of_dismiss_corners ++;
			}

		}
		
	}
	//if(action.width==3 && action.height==2)
	//{
	//	printf("计算角区完毕\n");
	//}
	return 4-2*num_of_dismiss_corners;
}


/*
	函数calculate_num_of_lines()
	参数：struct Action action
	返回值：double 
	功能：计算出本动作与动作空间的贴边数。

*/
int calculate_num_of_lines(struct Action action, int index_actionspace)
{
	struct ActionSpace actionspace;
	int rec_width,rec_height;


	actionspace = ACTION_SPACE[index_actionspace];

	if(action.pose == 0)//0表示躺着。1表示站着。
	{
		rec_width = action.width;
		rec_height = action.height;
	}
	else
	{
		rec_width = action.height;
		rec_height = action.width;
	}

	if(rec_width==actionspace.width && rec_height==actionspace.height)
		return 4;

	if(rec_width==actionspace.width || rec_height==actionspace.height)
		return 3;

	return 2;
	
}



/*
	函数current_action_is_better()
	参数：struct Action current_action, struct Action best_action
	返回值：int 
	功能：判断：若current_action比best_action更优，则返回1。否则返回0。

*/

int current_action_is_better(struct Action current_action, struct Action best_action)
{

	if(current_action.num_of_corners_increament < best_action.num_of_corners_increament)
		return 1;
	else if(current_action.num_of_corners_increament == best_action.num_of_corners_increament)
	{
		if(current_action.num_of_lines > best_action.num_of_lines)
			return 1;
		else if(current_action.num_of_lines == best_action.num_of_lines)
		{
			if(current_action.width+current_action.height > best_action.width+best_action.height)
				return 1;
			else if(current_action.width+current_action.height == best_action.width+best_action.height)
			{
				if(current_action.width > best_action.width)
					return 1;
				else if(current_action.width == best_action.width)
				{
					if(current_action.height > best_action.height)
						return 1;
					else if(current_action.height == best_action.height)
					{
						if(current_action.y < best_action.y)
							return 1;
						else if(current_action.y == best_action.y)
						{
							if(current_action.x < best_action.x)
								return 1;
							else if(current_action.x == best_action.x)
							{
								if(current_action.pose < best_action.pose)
									return 1;
							}
						}
					}
				}
			}
		}
	}
	
	return 0;

}

/*
	函数basic_algorithm_choose_an_action()
	参数：无
	返回值：返回-1表示失败，无动作可作。返回0表示选中了一个动作。
	功能：基本算法B0，选择一个动作
*/
int basic_algorithm_choose_an_action(struct Action * ptr_action, int * ptr_index_actionspace)
{
	int index_actionspace,index_rectangle,current_pose,rec_width,rec_height,corner_type,first_action_flag,first_rec_flag;
	struct ActionSpace current_actionspace;
	struct RectangleInfo current_rectangle,rectangle;
	struct Action current_action,best_action;
	int current_actionspace_valid_flag;

	//算出当前动作空间数组中的每个动作空间的每个角是否为90度角区
	//pointer_to_ACTION_SPACE是全局变量，其值为当前动作空间数组中存储的动作空间数。
	//index_actionspace为循环控制变量
	for(index_actionspace=0; index_actionspace<pointer_to_ACTION_SPACE; index_actionspace++)
	{
		current_actionspace = ACTION_SPACE[index_actionspace];
		if(current_actionspace.flag == 0)//无效的动作空间
		{
			continue;
		}

		//依次考虑本动作空间的四个角。调用current_corner_is_90_degree函数来做计算。
		for(corner_type=0; corner_type<=3; corner_type++)
		{
			//动作空间ACTION_SPACE[index_actionspace]的corner_90_degree字段是一个四元素的int型数组，
			//元素的值表示角是不是90度角区。1为是，0为否。
			ACTION_SPACE[index_actionspace].corner_90_degree[corner_type] = 
				current_corner_is_90_degree(current_actionspace, corner_type);
		}

	}

	first_action_flag = 1;//first_action_flag标记初始化为1，表示第一个被考虑的合理占角动作。
	for(index_actionspace=0; index_actionspace<pointer_to_ACTION_SPACE; index_actionspace++)
	{
		//循环，考虑每一个动作空间
		current_actionspace = ACTION_SPACE[index_actionspace];
		if(current_actionspace.flag == 0)//无效的动作空间不考虑
		{
			continue;
		}
		
		//如果当前动作空间，所有矩形块都放不下，则其current_actionspace_valid_flag标记值为0。
		//对current_actionspace_valid_flag变量设置初始值为0。
		current_actionspace_valid_flag = 0;
		
		first_rec_flag = 1;//相同形状大小的矩形块只考虑一个，标记first_rec_flag设初始值为1
		for(index_rectangle=0; index_rectangle<NUM_OF_RECTANGLES; index_rectangle++)
		{
			current_rectangle = RECTANGLES_INFO[index_rectangle];
			if(current_rectangle.flag == 1)	
				continue;	//木块已经在盒内，不考虑

			if(first_rec_flag == 1)	//若first_rec_flag值为1，则改为0，本矩形块current_rectangle记录在rectangle变量中。
			{
				first_rec_flag = 0;
				rectangle = current_rectangle;
			}
			else	//若first_rec_flag值为0，则看本矩形块current_rectangle，如果与rectangle完全相同，则这个矩形块不考虑。
			{
				if(current_rectangle.width == rectangle.width && current_rectangle.height == rectangle.height)
					continue;
				else//本矩形块current_rectangle，如果与rectangle不完全相同，则这个矩形块要考虑。
				{
					rectangle = current_rectangle;
				}
			}

			current_action.width = current_rectangle.width;		//矩形块的长边长
			current_action.height = current_rectangle.height;	//矩形块的短边长

			for(current_pose=0; current_pose<=1; current_pose++)//0表示木块躺着，1表示站着
			{
				if(current_pose == 0)
				{
					rec_width = current_rectangle.width;
					rec_height = current_rectangle.height;
				}
				else
				{
					rec_width = current_rectangle.height;
					rec_height = current_rectangle.width;
				}
				//rec_width表示本矩形块按照current_pose这种姿态放下后，在x轴方向上的长度。
				//rec_height表示本矩形块按照current_pose这种姿态放下后，在y轴方向上的长度。
	
				if(rec_width>current_actionspace.width || rec_height>current_actionspace.height)
					continue;	//矩形块按照current_pose这种姿态在本动作空间中放不下，则不考虑此动作。

				//肯定能放下，则current_actionspace_valid_flag标记置为1，表示本动作空间肯定有某个矩形块能放进去。
				current_actionspace_valid_flag = 1;
				//我们依次考虑动作空间的四个角。考虑四个占角动作。
				for(corner_type=0; corner_type<=3; corner_type++)
				{
					//current_action是当前考虑的占角动作。current_action.index字段表示所放矩形块的下标。
					current_action.index = index_rectangle;
					//current_action.pose表示动作的姿态
					current_action.pose = current_pose;

					//current_action.x和current_action.y表示这个占角动作，将矩形块放置的位置。矩形块的左下角的x，y坐标。
					if(corner_type==0)			//占动作空间的左下角
					{
						current_action.x = current_actionspace.x;
						current_action.y = current_actionspace.y;
					}
					else if(corner_type==1)		//占动作空间的右下角
					{
						current_action.x = current_actionspace.x + current_actionspace.width - rec_width;
						current_action.y = current_actionspace.y;
					}
					else if(corner_type==2)		//占动作空间的右上角
					{
						current_action.x = current_actionspace.x + current_actionspace.width - rec_width;
						current_action.y = current_actionspace.y + current_actionspace.height - rec_height;
					}
					else						//占动作空间的左上角
					{
						current_action.x = current_actionspace.x;
						current_action.y = current_actionspace.y + current_actionspace.height - rec_height;
					}

					//如果当前角（用corner_type表明）不是90度角区，则不考虑此动作。
					if(ACTION_SPACE[index_actionspace].corner_90_degree[corner_type] == 0)
						continue;

					//计算current_action的各项指标
					//计算current_action做完以后，角区数增加多少。
					current_action.num_of_corners_increament = calculate_num_of_corners_increament(current_action, corner_type);
					//计算current_action动作与其所在动作空间的贴边数
					current_action.num_of_lines = calculate_num_of_lines(current_action, index_actionspace);

					//2014.9.19. debug：current_action的 index_actionspace字段没有写。已改正。
					current_action.index_actionspace = index_actionspace;					
					
					//first_action_flag标记为1，表示第一个被考虑的合理占角动作。
					if(first_action_flag == 1)
					{
						//将first_action_flag标记设置为0
						first_action_flag = 0;
						//将当前动作记为best_action
						best_action = current_action;
						//记录当前动作所在的动作空间在动作空间数组中的下标
						* ptr_index_actionspace = index_actionspace;
					}
					else
					{
						//不是第一个被考虑的合理占角动作
						//如果current_action优于best_action，则将当前动作记为best_action。
						if(current_action_is_better(current_action, best_action) == 1)
						{
							best_action = current_action;
							//记录当前动作所在的动作空间在动作空间数组中的下标
							* ptr_index_actionspace = index_actionspace;
						}
					}


				}
			}
		}

		//如果current_actionspace的标记值为0，表示这个动作空间装不下任何矩形块。则将其标记为无效。
		if(current_actionspace_valid_flag == 0)
			ACTION_SPACE[index_actionspace].flag = 0;
	}

	
	if(first_action_flag == 1)
	{
		//当循环结束时，若first_action_flag值仍为1，表示当前格局下，没有合理占角动作可作。则函数返回-1。
		return -1;
	}
	else
	{
		//否则将best_action写入指针ptr_action指向的内存单元（结构体）。函数返回0。
		*ptr_action = best_action;
		return 0;		
	}
}

/*
	函数do_an_action()
	参数：struct Action action, int index_actionspace
	返回值：void
	功能：做一个动作，更新相关数据结构。
*/
void do_an_action(struct Action action, int index_actionspace)
{
	int index_rectangle,index_angle,i,index_edge,corner_type,x,y,flag,index_vertex,number_of_angles;
	int rectangle_x,rectangle_y,rectangle_pose,rectangle_width,rectangle_height,rec_width,rec_height;
	struct ActionSpace actionspace,new_actionspace1,new_actionspace2,new_actionspace3,new_actionspace4;
	int overlapx,overlapy,overlap;
	int x1,y1,x2,y2;
	struct Vertex temp_vertex;

	//更新相关数据结构

	//1. 矩形数组 ok
	index_rectangle = action.index;
	RECTANGLES_INFO[index_rectangle].x = action.x;
	RECTANGLES_INFO[index_rectangle].y = action.y;
	RECTANGLES_INFO[index_rectangle].flag = 1;
	RECTANGLES_INFO[index_rectangle].pose = action.pose;
	
	//2. 角数组 ok
	index_angle = pointer_to_ANGLE;
	for(i=0; i<=3; i++)
	{
		ANGLE[index_angle+i].index = action.index;
		ANGLE[index_angle+i].type = i;
	}
	pointer_to_ANGLE += 4;

	//3. 边数组  ok
	index_edge = pointer_to_EDGE;
	for(i=0; i<=3; i++)
	{
		EDGE[index_edge+i].index = action.index;
		EDGE[index_edge+i].type = i;
	}

	if(action.pose == 0)
	{
		rec_width = action.width;
		rec_height = action.height;
	}
	else
	{
		rec_width = action.height;
		rec_height = action.width;
	}
	//边的端点坐标(x1,y1)和(x2,y2)。x1<=x2, y1<=y2。
	EDGE[index_edge+0].x1 = action.x;				EDGE[index_edge+0].y1 = action.y;
	EDGE[index_edge+0].x2 = action.x+rec_width;		EDGE[index_edge+0].y2 = action.y;

	EDGE[index_edge+1].x1 = action.x+rec_width;		EDGE[index_edge+1].y1 = action.y;
	EDGE[index_edge+1].x2 = action.x+rec_width;		EDGE[index_edge+1].y2 = action.y+rec_height;

	EDGE[index_edge+2].x1 = action.x;				EDGE[index_edge+2].y1 = action.y+rec_height;
	EDGE[index_edge+2].x2 = action.x+rec_width;		EDGE[index_edge+2].y2 = action.y+rec_height;

	EDGE[index_edge+3].x1 = action.x;				EDGE[index_edge+3].y1 = action.y;
	EDGE[index_edge+3].x2 = action.x;				EDGE[index_edge+3].y2 = action.y+rec_height;

	pointer_to_EDGE += 4;

	//4. 顶点数组 
	//4.1 本动作对应矩形块的四个顶点。
	for(corner_type=0; corner_type<=3; corner_type++)
	{
		//依次考虑本动作对应矩形块的四个顶点。
		//计算出顶点坐标，分四种情况。
		if(corner_type==0)
		{
			x = action.x;
			y = action.y;
		}
		else if(corner_type==1)
		{
			x = action.x + rec_width; //2014.5.15 debug 顶点坐标更新： rec_width,不是action.width
			y = action.y;
		}
		else if(corner_type==2)
		{
			x = action.x + rec_width;
			y = action.y + rec_height;
		}
		else
		{
			x = action.x;
			y = action.y + rec_height;			
		}

		//看顶点是否在VERTEX数组中。
		//顺序查找程序段 begin---------------------------------------------------------------
		//flag = 0;
		//for(index_vertex=0; index_vertex<pointer_to_VERTEX; index_vertex++)
		//{
		//	if(x==VERTEX[index_vertex].x && y==VERTEX[index_vertex].y)
		//	{
		//		flag = 1; 
		//		break;
		//	}
		//}
		//顺序查找程序段 end-----------------------------------------------------------------
		//折半查找的程序段begin---------------------------------------------------------------
		index_vertex = half_search_vertex(VERTEX, pointer_to_VERTEX, x, y);
		if(index_vertex == -1)	//顶点数组中未找到，这是新顶点
			flag = 0;
		else
			flag = 1;
		//折半查找的程序段end---------------------------------------------------------------

		//flag为1：老顶点。0：新顶点。

		//若在VERTEX数组中，则老顶点增加一个角。老顶点不需要增加新的边。
		if(flag == 1)
		{	
			number_of_angles = VERTEX[index_vertex].number_of_angles;
			VERTEX[index_vertex].index_of_angles[number_of_angles] = pointer_to_ANGLE - (4-corner_type);
			VERTEX[index_vertex].number_of_angles ++;
		}
		else //若不在VERTEX数组中，则新顶点恰有一个角。
		{
			index_vertex = pointer_to_VERTEX;
			VERTEX[index_vertex].x = x;
			VERTEX[index_vertex].y = y;
			VERTEX[index_vertex].number_of_angles = 1;
			VERTEX[index_vertex].index_of_angles[0] = pointer_to_ANGLE - (4-corner_type);
			pointer_to_VERTEX ++;
			//排序
			while(index_vertex >= 1)
			{
				if(VERTEX[index_vertex-1].y < VERTEX[index_vertex].y ||
					VERTEX[index_vertex-1].y == VERTEX[index_vertex].y && VERTEX[index_vertex-1].x < VERTEX[index_vertex].x)
					break;
				else
				{
					//交换
					temp_vertex = VERTEX[index_vertex-1];
					VERTEX[index_vertex-1] = VERTEX[index_vertex];
					VERTEX[index_vertex] = temp_vertex;

					index_vertex --;
				}
			}


			//还要看看新顶点是否顶住一条边。
			//2014.9.20. debug: VERTEX[index_vertex].number_of_edge应该赋初值为0. 
			VERTEX[index_vertex].number_of_edge = 0;	
			
			//首先我们看框的四条边。
			//下边：
			if(y==0 && x>0 && x<BOX_WIDTH)
			{
				VERTEX[index_vertex].number_of_edge = 1;
				continue;
			}
			//右边
			if(x==BOX_WIDTH && y>0 && y<BOX_HEIGHT)
			{
				VERTEX[index_vertex].number_of_edge = 1;
				continue;
			}
			//上边
			if(y==BOX_HEIGHT && x>0 && x<BOX_WIDTH)
			{
				VERTEX[index_vertex].number_of_edge = 1;
				continue;
			}
			//左边
			if(x==0 && y>0 && y<BOX_HEIGHT)
			{
				VERTEX[index_vertex].number_of_edge = 1;
				continue;
			}

			//接下来，我们考虑其它在盒子中的矩形的4条边。代码同上面的代码类似。 
			//2014.9.21. debug: index_edge值为0-3的4条边是矩形框的边，其index值为-1。用-1做下标去RECTANGLES_INFO数组中找元素，可能会引起崩溃。
			//前几天运行时确实发生过崩溃。
			//for(index_edge=0; index_edge<pointer_to_EDGE; index_edge+=4)
			for(index_edge=4; index_edge<pointer_to_EDGE; index_edge+=4)
			{
				index_rectangle = EDGE[index_edge].index;
				rectangle_x = RECTANGLES_INFO[index_rectangle].x;
				rectangle_y = RECTANGLES_INFO[index_rectangle].y;
				rectangle_pose = RECTANGLES_INFO[index_rectangle].pose;
				if(rectangle_pose == 0)
				{
					rectangle_width = RECTANGLES_INFO[index_rectangle].width;
					rectangle_height = RECTANGLES_INFO[index_rectangle].height;
				}
				else
				{
					rectangle_width = RECTANGLES_INFO[index_rectangle].height;
					rectangle_height = RECTANGLES_INFO[index_rectangle].width;
				}
			
				//下边：
				if(y==rectangle_y && x>rectangle_x && x<rectangle_x+rectangle_width)
				{
					VERTEX[index_vertex].number_of_edge = 1;
					break;
				}
				//右边
				if(x==rectangle_x+rectangle_width && y>rectangle_y && y<rectangle_y+rectangle_height)
				{
					VERTEX[index_vertex].number_of_edge = 1;
					break;
				}
				//上边
				if(y==rectangle_y+rectangle_height && x>rectangle_x && x<rectangle_x+rectangle_width)
				{
					VERTEX[index_vertex].number_of_edge = 1;
					break;
				}
				//左边
				if(x==rectangle_x && y>rectangle_y && y<rectangle_y+rectangle_height)
				{
					VERTEX[index_vertex].number_of_edge = 1;
					break;
				}

			}
		}
		
	}

	//4.2 2014.9.14. debug：应该考虑。顶点数组中的那些顶点，是否顶住本动作对应矩形块的四条边。（顶点恰好位于边的端点的，不算。）
	for(index_edge=pointer_to_EDGE-4; index_edge<=pointer_to_EDGE-1; index_edge++)
	{
		x1 = EDGE[index_edge].x1;	y1 = EDGE[index_edge].y1;
		x2 = EDGE[index_edge].x2;	y2 = EDGE[index_edge].y2;

		for(index_vertex=0; index_vertex<=pointer_to_VERTEX; index_vertex++)
		{
			//若顶点被边顶住，则记下来。
			if(index_edge %2 ==0)	//横边
			{
				if(VERTEX[index_vertex].y == y1 && VERTEX[index_vertex].x > x1  && VERTEX[index_vertex].x < x2)
				{
					VERTEX[index_vertex].number_of_edge = 1;
				}
			}
			else					//竖边
			{
				if(VERTEX[index_vertex].x == x1 && VERTEX[index_vertex].y > y1  && VERTEX[index_vertex].y < y2)
				{
					VERTEX[index_vertex].number_of_edge = 1;
				}				
			}
		}
	}

	//5. 动作空间  
 
	//5.1 更新其所在的动作空间。
	actionspace = ACTION_SPACE[index_actionspace];
	if(action.pose == 0)
	{
		rec_width = action.width;
		rec_height = action.height;
	}
	else
	{
		rec_width = action.height;
		rec_height = action.width;
	}
	
	new_actionspace1.width = actionspace.width;
	new_actionspace1.height = actionspace.height - rec_height;
	if(action.y == actionspace.y)//动作占动作空间的下两角。
	{
		new_actionspace1.x = actionspace.x;
		new_actionspace1.y = actionspace.y + rec_height;
	}
	else
	{
		new_actionspace1.x = actionspace.x;
		new_actionspace1.y = actionspace.y;		
	}
	new_actionspace1.flag = 1;

	new_actionspace2.width = actionspace.width - rec_width;
	new_actionspace2.height = actionspace.height;
	if(action.x == actionspace.x)//动作占动作空间的左两角。
	{
		new_actionspace2.x = actionspace.x + rec_width;
		new_actionspace2.y = actionspace.y;
	}
	else
	{
		new_actionspace2.x = actionspace.x;
		new_actionspace2.y = actionspace.y;		
	}
	new_actionspace2.flag = 1;

	//更新动作空间数组，分四种情况讨论。
	if(new_actionspace1.height>0 && new_actionspace2.width>0)
	{	
		ACTION_SPACE[index_actionspace] = new_actionspace1;//应该写new_actionspace1的undo字段
		ACTION_SPACE[pointer_to_ACTION_SPACE ++] = new_actionspace2;
	}
	else if(new_actionspace1.height>0 && new_actionspace2.width<=0)
	{
		ACTION_SPACE[index_actionspace] = new_actionspace1;
	}
	else if(new_actionspace1.height<=0 && new_actionspace2.width > 0)
	{
		ACTION_SPACE[index_actionspace] = new_actionspace2;
	}
	else
	{
		//删去此动作空间
		ACTION_SPACE[index_actionspace].flag = 0;
	}

	//5.2 看本动作是否与动作空间数组中其它动作空间重叠。若有重叠，则需要更新。
	for(i=0; i<pointer_to_ACTION_SPACE; i++)
	{
		actionspace = ACTION_SPACE[i];
		//判断action与actionspace是否重叠。
		if(action.x>=actionspace.x+actionspace.width || actionspace.x>=action.x+rec_width)//不重叠
			overlapx = 0;
		else
			overlapx = 1;

		if(action.y>=actionspace.y+actionspace.height || actionspace.y>=action.y+rec_height)
			overlapy = 0;
		else
			overlapy = 1;

		if(overlapx==1 && overlapy==1)
			overlap = 1;
		else
			overlap = 0;

		if(overlap == 0)//不重叠，不需要考虑更新动作空间。
			continue;


		//考虑如何更新动作空间。
		//若此动作完全覆盖动作空间，则删去此动作空间。
		if(action.x<=actionspace.x && action.x+rec_width>=actionspace.x+actionspace.width &&
			action.y<=actionspace.y && action.y+rec_height>=actionspace.y+actionspace.height)
		{
			ACTION_SPACE[i].flag = 0;
			//INDEX_DELETE_ACTION_SPACE[pointer_to_INDEX_DELETE_ACTION_SPACE ++] = i;
			continue;
		}

		if(action.x>actionspace.x && action.x+rec_width<actionspace.x+actionspace.width &&
			action.y>actionspace.y && action.y+rec_height<actionspace.y+actionspace.height)
		{
			//列举出四个新的动作空间
			new_actionspace1.width = actionspace.width;
			new_actionspace1.height = (actionspace.y+actionspace.height) - (action.y+rec_height);
			new_actionspace1.x = actionspace.x;
			new_actionspace1.y = action.y + rec_height;
			new_actionspace1.flag = 1;

			new_actionspace2.width = actionspace.width;
			new_actionspace2.height = action.y - actionspace.y;
			new_actionspace2.x = actionspace.x;
			new_actionspace2.y = actionspace.y;
			new_actionspace2.flag = 1;

			new_actionspace3.width = (actionspace.x+actionspace.width) - (action.x+rec_width);
			new_actionspace3.height = actionspace.height;
			new_actionspace3.x = action.x + rec_width;
			new_actionspace3.y = actionspace.y;
			new_actionspace3.flag = 1;

			new_actionspace4.width = action.x - actionspace.x;
			new_actionspace4.height = actionspace.height;
			new_actionspace4.x = actionspace.x;
			new_actionspace4.y = actionspace.y;
			new_actionspace4.flag = 1;

			//写入动作空间数组
			ACTION_SPACE[i] = new_actionspace1;
			ACTION_SPACE[pointer_to_ACTION_SPACE ++] = new_actionspace2;
			ACTION_SPACE[pointer_to_ACTION_SPACE ++] = new_actionspace3;
			ACTION_SPACE[pointer_to_ACTION_SPACE ++] = new_actionspace4;
		}
		else if( (action.x<=actionspace.x || action.x+rec_width>=actionspace.x+actionspace.width) &&
			action.y>actionspace.y && action.y+rec_height<actionspace.y+actionspace.height)
		{
			//列举出三个或两个新的动作空间
			
			new_actionspace1.width = actionspace.width;
			new_actionspace1.height = (actionspace.y+actionspace.height) - (action.y+rec_height);
			new_actionspace1.x = actionspace.x;
			new_actionspace1.y = action.y + rec_height;
			new_actionspace1.flag = 1;

			new_actionspace2.width = actionspace.width;
			new_actionspace2.height = action.y - actionspace.y;
			new_actionspace2.x = actionspace.x;
			new_actionspace2.y = actionspace.y;
			new_actionspace2.flag = 1;
			
			if(action.x<=actionspace.x && action.x+rec_width>=actionspace.x+actionspace.width)
			{
				//写入动作空间数组
				
				ACTION_SPACE[i] = new_actionspace1;
				ACTION_SPACE[pointer_to_ACTION_SPACE ++] = new_actionspace2;				
			}
			else if(action.x<=actionspace.x && action.x+rec_width<actionspace.x+actionspace.width)
			{
				new_actionspace3.width = (actionspace.x+actionspace.width) - (action.x+rec_width);
				new_actionspace3.height = actionspace.height;
				new_actionspace3.x = action.x + rec_width;
				new_actionspace3.y = actionspace.y;
				new_actionspace3.flag = 1;

				//写入动作空间数组
				

				ACTION_SPACE[i] = new_actionspace1;
				ACTION_SPACE[pointer_to_ACTION_SPACE ++] = new_actionspace2;
				ACTION_SPACE[pointer_to_ACTION_SPACE ++] = new_actionspace3;
			}
			else//action.x>actionspace.x && action.x+rec_width>=actionspace.x+actionspace.width
			{
				new_actionspace3.width = action.x - actionspace.x;
				new_actionspace3.height = actionspace.height;
				new_actionspace3.x = actionspace.x;
				new_actionspace3.y = actionspace.y;
				new_actionspace3.flag = 1;

				//写入动作空间数组
				

				ACTION_SPACE[i] = new_actionspace1;
				ACTION_SPACE[pointer_to_ACTION_SPACE ++] = new_actionspace2;
				ACTION_SPACE[pointer_to_ACTION_SPACE ++] = new_actionspace3;
			}
		}
		else if(action.x>actionspace.x && action.x+rec_width<actionspace.x+actionspace.width &&
			(action.y<=actionspace.y || action.y+rec_height>=actionspace.y+actionspace.height) )
		{
			//列举出三个或两个新的动作空间
			new_actionspace3.width = (actionspace.x+actionspace.width) - (action.x+rec_width);
			new_actionspace3.height = actionspace.height;
			new_actionspace3.x = action.x + rec_width;
			new_actionspace3.y = actionspace.y;
			new_actionspace3.flag = 1;

			new_actionspace4.width = action.x - actionspace.x;
			new_actionspace4.height = actionspace.height;
			new_actionspace4.x = actionspace.x;
			new_actionspace4.y = actionspace.y;
			new_actionspace4.flag = 1;
			
			if(action.y<=actionspace.y && action.y+rec_height>=actionspace.y+actionspace.height)
			{
				//写入动作空间数组
				

				ACTION_SPACE[i] = new_actionspace3;
				ACTION_SPACE[pointer_to_ACTION_SPACE ++] = new_actionspace4;	
			}
			if(action.y<=actionspace.y && action.y+rec_height<actionspace.y+actionspace.height)
			{
				new_actionspace1.width = actionspace.width;
				new_actionspace1.height = (actionspace.y+actionspace.height) - (action.y+rec_height);
				new_actionspace1.x = actionspace.x;
				new_actionspace1.y = action.y + rec_height;
				new_actionspace1.flag = 1;

				//写入动作空间数组
				

				ACTION_SPACE[i] = new_actionspace1;
				ACTION_SPACE[pointer_to_ACTION_SPACE ++] = new_actionspace3;
				ACTION_SPACE[pointer_to_ACTION_SPACE ++] = new_actionspace4;
			}
			else
			{
				new_actionspace1.width = actionspace.width;
				new_actionspace1.height = action.y - actionspace.y;
				new_actionspace1.x = actionspace.x;
				new_actionspace1.y = actionspace.y;
				new_actionspace1.flag = 1;

				//写入动作空间数组
				

				ACTION_SPACE[i] = new_actionspace1;
				ACTION_SPACE[pointer_to_ACTION_SPACE ++] = new_actionspace3;
				ACTION_SPACE[pointer_to_ACTION_SPACE ++] = new_actionspace4;
			}
		}
		else //(action.x<=actionspace.x || action.x+rec_width>=actionspace.x+actionspace.width) && (action.y<=actionspace.y || action.y+rec_height>=actionspace.y+actionspace.height)	
		{
			if(action.x<=actionspace.x && action.x+rec_width>=actionspace.x+actionspace.width)
			{
				//只有1个新的动作空间
				if(action.y<=actionspace.y && action.y+rec_height<actionspace.y+actionspace.height)
				{
					new_actionspace1.width = actionspace.width;
					new_actionspace1.height = (actionspace.y+actionspace.height) - (action.y+rec_height);
					new_actionspace1.x = actionspace.x;
					new_actionspace1.y = action.y + rec_height;
					new_actionspace1.flag = 1;
					//更新动作空间数组
					

					ACTION_SPACE[i] = new_actionspace1;
				}
				else//action.y>actionspace.y && action.y+rec_height>=actionspace.y+actionspace.height
				{
					new_actionspace1.width = actionspace.width;
					new_actionspace1.height = action.y - actionspace.y;
					new_actionspace1.x = actionspace.x;
					new_actionspace1.y = actionspace.y;
					new_actionspace1.flag = 1;
					//更新动作空间数组
					

					ACTION_SPACE[i] = new_actionspace1;
				}
			}
			else if(action.y<=actionspace.y && action.y+rec_height>=actionspace.y+actionspace.height)
			{
				//只有1个新的动作空间
				if(action.x<=actionspace.x && action.x+rec_width<actionspace.x+actionspace.width)
				{
					new_actionspace3.width = (actionspace.x+actionspace.width) - (action.x+rec_width);
					new_actionspace3.height = actionspace.height;
					new_actionspace3.x = action.x + rec_width;
					new_actionspace3.y = actionspace.y;
					new_actionspace3.flag = 1;
					//更新动作空间数组
					

					ACTION_SPACE[i] = new_actionspace3;
				}
				else//action.x>actionspace.x && action.x+rec_width>=actionspace.x+actionspace.width
				{
					new_actionspace3.width = action.x - actionspace.x;
					new_actionspace3.height = actionspace.height;
					new_actionspace3.x = actionspace.x;
					new_actionspace3.y = actionspace.y;
					new_actionspace3.flag = 1;
					//更新动作空间数组
					

					ACTION_SPACE[i] = new_actionspace3;
				}
			}
			else
			{
				//列举出两个新的动作空间
				if(action.y<=actionspace.y)
				{
					new_actionspace1.width = actionspace.width;
					new_actionspace1.height = (actionspace.y+actionspace.height) - (action.y+rec_height);
					new_actionspace1.x = actionspace.x;
					new_actionspace1.y = action.y + rec_height;
					new_actionspace1.flag = 1;
				}
				else
				{
					new_actionspace1.width = actionspace.width;
					new_actionspace1.height = action.y - actionspace.y;
					new_actionspace1.x = actionspace.x;
					new_actionspace1.y = actionspace.y;
					new_actionspace1.flag = 1;
				}

				if(action.x<=actionspace.x)
				{
					new_actionspace3.width = (actionspace.x+actionspace.width) - (action.x+rec_width);
					new_actionspace3.height = actionspace.height;
					new_actionspace3.x = action.x + rec_width;
					new_actionspace3.y = actionspace.y;
					new_actionspace3.flag = 1;
				}
				else
				{
					new_actionspace3.width = action.x - actionspace.x;
					new_actionspace3.height = actionspace.height;
					new_actionspace3.x = actionspace.x;
					new_actionspace3.y = actionspace.y;
					new_actionspace3.flag = 1;
				}

				//写入动作空间数组
				

				ACTION_SPACE[i] = new_actionspace1;
				ACTION_SPACE[pointer_to_ACTION_SPACE ++] = new_actionspace3;
			}
		}


	}

	
	
}


/*
	函数delete_invailid_and_reluctant_action_space()
	参数：void
	返回值：void
	功能：删去无效的动作空间。删去多余的动作空间，有的动作空间被其它动作空间包含，则为多余，删去。
*/
void delete_invailid_and_reluctant_action_space()
{
	int i,j,contian_flag;


	//删去无效的动作空间。flag值为0的动作空间是无效的。
	i = 0;
	j = 0;
	while(1)
	{
		if(i >= pointer_to_ACTION_SPACE)
			break;

		if(ACTION_SPACE[i].flag == 1)
		{
			if(i != j)
			{
				ACTION_SPACE[j++] = ACTION_SPACE[i];
			}
			else
				j++;
		}
		i++;
	}
	pointer_to_ACTION_SPACE = j;


	//处理有包含关系的动作空间
	i = 0;
	while(1)
	{
		if(i >= pointer_to_ACTION_SPACE)
			break;

		//看i是否被其它动作空间包含。若是，则contain_flag=1，若否，则contain_flag=0.
		contian_flag = 0;
		for(j=0; j<pointer_to_ACTION_SPACE; j++)
		{
			if(j == i)
				continue;
			if(ACTION_SPACE[j].flag == 0)
				continue;
			//若j包含i，则contian_flag=1. break;
			if(ACTION_SPACE[j].x <= ACTION_SPACE[i].x && ACTION_SPACE[j].x+ACTION_SPACE[j].width >= ACTION_SPACE[i].x+ACTION_SPACE[i].width &&
				ACTION_SPACE[j].y <= ACTION_SPACE[i].y && ACTION_SPACE[j].y+ACTION_SPACE[j].height >= ACTION_SPACE[i].y+ACTION_SPACE[i].height)
			{
				contian_flag=1; 
				break;
			}
		}

		if(contian_flag == 0)
		{
			i++;
		}
		else
		{
			//删去i
			//for(j=i; j<pointer_to_ACTION_SPACE-1; j++)
			//	ACTION_SPACE[j] = ACTION_SPACE[j+1];
			//pointer_to_ACTION_SPACE--;
			ACTION_SPACE[i].flag = 0;
			i++;
		}
	}

	//删去无效的动作空间。flag值为0的动作空间是无效的。
	i = 0;
	j = 0;
	while(1)
	{
		if(i >= pointer_to_ACTION_SPACE)
			break;

		if(ACTION_SPACE[i].flag == 1)
		{
			if(i != j)
			{
				ACTION_SPACE[j++] = ACTION_SPACE[i];
			}
			else
				j++;
		}
		i++;
	}
	pointer_to_ACTION_SPACE = j;


}


int main(void)
{
	char file_name[100];
	FILE * fp;
	int i;
	double ratio;
	struct Action action;
	int index_actionspace;
	int sum_of_area;
	int t=0;
	clock_t start, end; 
	int dis,x1,y1,x2,y2,rec_width,rec_height;
	int LB;

	
	//1. 读入数据
	//算例的数据从文件读入      盒子高度值从文件读入，对于c1.txt来说，盒子宽度值定为20.

	//计算当前工程的c文件夹中的c1.txt这个算例
	strcpy(file_name, "c\\c1.txt");  //如果要计算其他算例，可以修改这一行。例如改为：c\\c4.txt。
	fp = fopen(file_name, "r");
	if(fp==NULL)
	{
		printf("Can not open the file.\n");
		return -1;
	}

	build_an_instance(fp);

	printf("木块数=%d\n", NUM_OF_RECTANGLES);
	//printf("盒子宽度=%d\n", BOX_WIDTH);
	printf("盒子高度=%d\n", BOX_HEIGHT);
	//for(i=0; i<NUM_OF_RECTANGLES; i++)
	//{
	//	printf("%d %d %d\n", i, WIDTHS_OF_RECTANGLES[i], HEIGHTS_OF_RECTANGLES[i]);
	//}

	fclose(fp);

	start = clock(); 

	//2. 计算
	init_data();
	LB = (int)ceil((double)SUM_OF_REC_AREA/(double)BOX_HEIGHT);	
	BOX_WIDTH = LB;	
	while(1)
	{
		init_data();
		LB = (int)ceil((double)SUM_OF_REC_AREA/(double)BOX_HEIGHT);	
		printf("LB=%d\n", LB);	
		
		printf("BOX_WIDTH=%d\n",BOX_WIDTH);

		while(1)
		{
		
			delete_invailid_and_reluctant_action_space();
			
			if(basic_algorithm_choose_an_action(&action, &index_actionspace) == -1)
			{
				break;
			}
			else
			{
				do_an_action(action, index_actionspace);
				ACTION_SEQ[pointer_to_ACTION_SEQ++] = action;
			}
		}

		//输出结果
		sum_of_area = 0;
		for(i=0; i<NUM_OF_RECTANGLES; i++)
		{
			if(RECTANGLES_INFO[i].flag == 1)
				sum_of_area += RECTANGLES_INFO[i].width*RECTANGLES_INFO[i].height;
		}

		ratio = (double)sum_of_area / (double)(BOX_WIDTH*BOX_HEIGHT);

		end = clock(); 

		printf("利用面积=%d\n", sum_of_area);

		printf("利用率为：%.2f\n", 100*ratio);

		ratio = (double)sum_of_area / (double)(SUM_OF_REC_AREA);

		printf("木块总面积=%d\n",SUM_OF_REC_AREA);

		printf("进入盒子的木块总面积=%d\n",sum_of_area);

		printf("木块完成率为：%.2f\n", 100*ratio);
	
		printf("计算时间为：%f\n", (double)(end - start) / CLK_TCK);
	
		if(sum_of_area == SUM_OF_REC_AREA)
		{
			printf("木块全部放入盒子\n");
			break;
		}
		BOX_WIDTH += 1;
	}
	//return 0;

	//getchar();

	//显示终止格局的示意图
	dis = 30;
	initgraph(1240, 640);

	
	//显示矩形框
	rectangle(0,BOX_HEIGHT*dis, BOX_WIDTH*dis,0);

	//for(i=0; i<NUM_OF_RECTANGLES; i++)
	//{
	//	if(RECTANGLES_INFO[i].flag == 0)
	//		continue;

	//	if(RECTANGLES_INFO[i].pose == 0)
	//	{
	//		rec_width = RECTANGLES_INFO[i].width;
	//		rec_height = RECTANGLES_INFO[i].height;
	//	}
	//	else
	//	{
	//		rec_width = RECTANGLES_INFO[i].height;
	//		rec_height = RECTANGLES_INFO[i].width;			
	//	}
	//	x1 = RECTANGLES_INFO[i].x;
	//	y1 = RECTANGLES_INFO[i].y + rec_height;
	//	x2 = RECTANGLES_INFO[i].x + rec_width;
	//	y2 = RECTANGLES_INFO[i].y;
		
	//	Sleep(3000);
	//	rectangle(x1*dis,y1*dis,x2*dis,y2*dis);		
	//}

	
	for(i=0; i<pointer_to_ACTION_SEQ; i++)
	{
		action = ACTION_SEQ[i];

		if(action.pose == 0)
		{
			rec_width = action.width;
			rec_height = action.height;
		}
		else
		{
			rec_width = action.height;
			rec_height = action.width;			
		}
		x1 = action.x;
		y1 = action.y + rec_height;
		x2 = action.x + rec_width;
		y2 = action.y;
		
		Sleep(3000);
		bar(x1*dis,y1*dis,x2*dis,y2*dis);
		setcolor(RED);
		rectangle(x1*dis,y1*dis,x2*dis,y2*dis);		
	}

	//fflush(stdin);
	getchar();
	
	closegraph();
	//3. 释放内存，结束。
	free_mem();


	return 0;
}

