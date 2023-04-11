
/*
�汾������Ϣ
2013.12.19. ��ʼ˼����ά����װ�����⡣

2014.5.17. �����㷨B0��һ�����д�꣬������ΪRecPackingB0

2014.5.26. Ѩ��ö���㷨B1��һ�����д�꣬������ΪRecPackingB1

2014.5.29. ������ö���㷨B2��һ�����д�꣬������ΪRecPackingB2

2014.9.3.  ������ö���㷨�ڶ������RecPackingB2bд�꣬���㷨�ͱ��������ڵ�һ���нϴ�Ľ���

2014.9.10. ��ʼд������ö���㷨���������RecPackingB2c����Ҫ��Ϊ�˳����߼�������ע�����ơ�

2014.9.13. ��ʼд������ö���㷨���İ����RecPackingB2d��
calculate_num_of_corners_increament��������Ҫ����VERTEX���飬��˳����Ҹ�Ϊ�۰���ҡ�
do_an_action������current_cornet_is_90_degree������ͬ����Ҫ����VERTEX���飬��˳����Ҹ�Ϊ�۰���ҡ�

2014.9.14. debug��Ӧ�ÿ��ǡ����������е���Щ���㣬�Ƿ�ס��������Ӧ���ο�������ߡ�������ǡ��λ�ڱߵĶ˵�ģ����㡣��

2014.9.15. �����㷨B0�ڶ����㷨д�꣬��ΪRecPackingB0a��ɾȥ���õ����ݽṹ�ͺ���������ע�ͣ��Ľ�����

2014.9.15. RecPackingB0aL����ΪRecPackingB0a�����޸Ķ������������ڼ���strip packing����ʵ���Ϸ��㡣

2014.9.20. debug��VERTEX[index_vertex].number_of_edgeӦ�ø���ֵΪ0��
������ö���㷨��������RecPackingB2eд�ꡣRecPackingB2eL1��RecPackingB2e�����������޸ġ����ڼ���strip packing���⡣

2014.9.20. �����㷨B0�������㷨д�꣬��ΪRecPackingB0b��ɾȥ���õ����ݽṹ�ͺ������Ľ�����ȥ��bug��

2014.9.20. RecPackingB0bL����ΪRecPackingB0b�����޸Ķ������������ڼ���strip packing����ʵ���Ϸ��㡣

������������ڽ�ѧ����������ҵ��;����������ͬ�⡣
���ߣ�����   �人�Ƽ���ѧ�������ѧ�뼼��ѧԺ �������ϵ  email:wanglei77@wust.edu.cn
Copyright: Wang Lei. All rights reserved.
*/

/*
�ο����ף�
[1] ����,������. ����ά����Packing�����һ��������ö���㷨. �й���ѧ. 2014. ��Ͷ�壬��һ��������ϡ�
[2] ����,������,����. ���ڶ����ռ�����ά����Packing����ĸ�Ч�㷨. ���ѧ��. 2012, 23(12):1037-1044
[3] Defu Zhang, Lijun Wei, Stephen C.H.Leung, Qingshan Chen. A binary search heuristic algorithm based on randomized
local search for the rectangular strip-packing problem. INFORMS Journal on Computing. 2013, 25(2):332-345
*/

#include <stdio.h>		//printf������Ҫ����stdio.h
#include <stdlib.h>		//malloc������free������Ҫ����stdlib.h
#include <string.h>		//strcpy������Ҫ����string.h
#include <time.h>		//clock������Ҫ����time.h

//graphics.hͷ�ļ�������ʾͼ��
#include <graphics.h>
#include <windows.h>

#include <math.h>



//--------------------------------------------------------------------------------------------------------
//�ṹ��Ķ���
//--------------------------------------------------------------------------------------------------------

//����1.��֣��ɽṹ��RectangleInfo��ʾ��
//struct RectangleInfo��ľ��������Ϣ��Ҫ˵���ľ���Ƿ��ں��ڡ����ں��ڣ�����̬�����ꡣ
struct RectangleInfo
{
	int index;	//ľ�����š�ͬʱҲ�Ǵ�ľ����ľ���Ⱥ�ľ��߶������е��±�
	int	flag;	//0��ʾ�ں��⡣1��ʾ�ں��ڡ�
	int width;	//ľ����  ���߳�
	int height;	//ľ��߶�  �̱߳�
	int x;		//ľ�����½�x����
	int y;		//ľ�����½�y����
	int pose;	//0��ʾ���š�1��ʾվ�š�
};

//����2.����
struct Action
{
	int	index;	//ľ�����š�ͬʱҲ�Ǵ�ľ����ľ���Ⱥ�ľ��߶������е��±�		�ĸ�ľ�飿
	int x;		//ľ�����½�x����													ʲôλ�ã�
	int y;		//ľ�����½�y����
	int pose;	//0��ʾ���š�1��ʾվ�š�											������̬��
	int index_actionspace;					//�������ĸ������ռ��У�
	
	int width;	//ľ����  ���߳�
	int height;	//ľ��߶�  �̱߳�

	int num_of_lines;						//�����붯���ռ��������
	int num_of_corners_increament;			//���������Ժ�ʣ��ռ�Ľ�����������ֵ��
	int beauty_degree;						//������������
};

//����3.�����ռ�
struct ActionSpace
{
	int x;		//�����ռ����½�x����
	int y;		//�����ռ����½�y����
	int width;	//�����ռ���  x��������
	int height;	//�����ռ�߶�  y��������

	int flag;	//1��ʾ��Ч��0��ʾ��Ч��

	int corner_90_degree[4];	//�����ռ����¡����¡����ϡ����ϣ����±�0��1��2��3��ʾ���ĸ����Ƿ�Ϊ90�Ƚ�����ȡֵ��0�ǣ�1��
};

//����4.ľ��Ľ�
struct Angle
{
	int index;	//�������ĸ�ľ�飬ľ����ľ���Ⱥ�ľ��߶������е��±ꡣ�±�-1��ʾ�����ľ�飬�����ο�
	int type;	//�ǵ����ͣ�0-3���ֱ��ʾ���¡����¡����ϡ����ϡ�
};

//����5.ľ��ı�
struct Edge
{
	int index;	//�������ĸ�ľ�飬ľ����ľ���Ⱥ�ľ��߶������е��±ꡣ�±�-1��ʾ�����ľ�飬�����ο�
	int type;	//�ߵ����ͣ�0-3���ֱ��ʾ�¡��ҡ��ϡ���

	int x1;		//�ߵ������˵�����꣬(x1,y1)��(x2,y2)
	int y1;
	int x2;
	int y2;
};

//����6.����
struct Vertex
{
	int x;					//��������(x,y)
	int y;

	int number_of_angles;	//�ж��ٸ�ľ���������������㡣ֵΪ1��4
	int index_of_angles[4];	//���ڽ������е��±�
	
	int number_of_edge;		//�ж������߾���������㡣�ߵĶ˵����������Ĳ��㡣 ֵΪ0��1
};



//----------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------


//============================================================================================================================
//============================================ȫ�ֱ���========================================================================
int BOX_WIDTH,BOX_HEIGHT;		//���ӵĿ�Ⱥ͸߶�
int NUM_OF_RECTANGLES;			//ľ����
int * WIDTHS_OF_RECTANGLES;		//ȫ�ֱ���WIDTHS_OF_RECTANGLES��ľ�������顣WIDTHS_OF_RECTANGLES[i]��ֵΪi��ľ��Ŀ�ȡ�
int * HEIGHTS_OF_RECTANGLES;	//ȫ�ֱ���HEIGHTS_OF_RECTANGLES��ľ��߶����顣HEIGHTS_OF_RECTANGLES[i]��ֵΪi��ľ��ĸ߶ȡ�

struct RectangleInfo * RECTANGLES_INFO;	//ľ����Ϣ���顣��ʾ��֡�

struct ActionSpace * ACTION_SPACE;		//�����ռ����顣��¼��ǰ����У����ж����ռ����Ϣ��
int pointer_to_ACTION_SPACE;			//pointer_to_ACTION_SPACE��ֵΪ��ǰ����У������ռ������

struct Angle * ANGLE;					//���ν����顣��¼��ǰ����У����з�����ο��ľ��Ľǵ���Ϣ��
int pointer_to_ANGLE;					//pointer_to_ANGLE��ֵΪ��ǰ����У�������ο��ľ��ĽǸ�����

struct Edge * EDGE;						//���α����顣��¼��ǰ����У����з�����ο��ľ��ıߵ���Ϣ��	
int pointer_to_EDGE;					//pointer_to_EDGE��ֵΪ��ǰ����У�������ο��ľ��ı�����

struct Vertex * VERTEX;					//�������顣��¼��ǰ����У����з�����ο��ľ���Լ����ο���Ķ�����Ϣ��
int pointer_to_VERTEX;					//pointer_to_VERTEX��ֵΪ��ǰ����еĶ����������غϵĶ�����һ����



struct Action * ACTION_SEQ;//�����Ķ�������
int pointer_to_ACTION_SEQ;

int SUM_OF_REC_AREA;	//ľ�������

//=============================================================================================================================


//===================================================================================================================
//==============================================����˵��=============================================================
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
	����build_an_instance()
	������FILE * fp
	����ֵ��void 
	���ܣ�1.����fscanf�⺯����ȡbenchmark�����ļ��е����ݡ�
	2.����malloc�⺯�������ڴ档

  benchmark�����ļ��ĸ�ʽ����˵�����£�
  3   ľ����
  20  ���Ӹ߶�
  0  3  5      ��һ��ľ����Ϊ3���߶�Ϊ5�����е����������ֱ��ʾľ����š�ľ���ȡ�ľ��߶ȡ�ע��ľ���0��ʼ��š�
  1  4  4      �������ƣ��ڶ���ľ��
  2  6  3      ������ľ��

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
	����init_data()
	��������
	����ֵ��void 
	���ܣ���ʼ�����ݽṹ

*/
void init_data()
{
	int i,j,t;

	//0. Ҫ��ľ���ȴ��ڵ���ľ��߶ȡ����ľ����С��ľ���ȣ���ľ���Ⱥ͸߶�ֵ��������������Ϊ�˼��㷽�㡣
	for(i=0; i<NUM_OF_RECTANGLES; i++)
	{
		if(WIDTHS_OF_RECTANGLES[i] < HEIGHTS_OF_RECTANGLES[i])
		{
			t = WIDTHS_OF_RECTANGLES[i];
			WIDTHS_OF_RECTANGLES[i] = HEIGHTS_OF_RECTANGLES[i];
			HEIGHTS_OF_RECTANGLES[i] = t;
		}
	}

	//1. ��ľ�������������Ϊ<�ܳ������ţ������߳������ţ����̱߳������ţ����±꣨С�ţ�>
	//��Ҫ���ú���rectangle_is_prior��
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

	//2. RECTANGLES_INFO�����ʼ��������ľ����ں������档
	for(i=0; i<NUM_OF_RECTANGLES; i++)
	{
		RECTANGLES_INFO[i].index = i;
		RECTANGLES_INFO[i].flag = 0;
		RECTANGLES_INFO[i].width = WIDTHS_OF_RECTANGLES[i];
		RECTANGLES_INFO[i].height = HEIGHTS_OF_RECTANGLES[i];
	}

	//3. ACTION_SPACE�����ʼ��
	ACTION_SPACE[0].x = 0;
	ACTION_SPACE[0].y = 0;
	ACTION_SPACE[0].width = BOX_WIDTH;
	ACTION_SPACE[0].height = BOX_HEIGHT;
	ACTION_SPACE[0].flag = 1;
	pointer_to_ACTION_SPACE = 1;

	//4. ANGLE�����ʼ��
	ANGLE[0].index = -1;
	ANGLE[0].type = 0;
	ANGLE[1].index = -1;
	ANGLE[1].type = 1;
	ANGLE[2].index = -1;
	ANGLE[2].type = 2;
	ANGLE[3].index = -1;
	ANGLE[3].type = 3;
	pointer_to_ANGLE = 4;

	//5. EDGE�����ʼ��
	//�ߵĶ˵�����(x1,y1)��(x2,y2)��x1<=x2, y1<=y2��
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

	//6. VERTEX�����ʼ��
	//VERTEX��������������<y��С���ȣ���x��С���ȣ�>
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


	//7.�������������ʼ��
	pointer_to_ACTION_SEQ = 0;

	//8.������ž��ο������
	SUM_OF_REC_AREA = 0;
	for(i=0; i<NUM_OF_RECTANGLES; i++)
	{
		SUM_OF_REC_AREA += RECTANGLES_INFO[i].width * RECTANGLES_INFO[i].height;
	}

	
}


/*
	����rectangle_is_prior()
	������int i, int j
	����ֵ��int 
	���ܣ��ж��±�Ϊi�ľ����Ƿ���±�Ϊj�ľ������ȣ����ǣ��򷵻�1���������򷵻�0��
	��init_data�������á�

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
	����free_mem()
	��������
	����ֵ��void 
	���ܣ�����free�⺯���ͷ�֮ǰ��malloc�⺯��������ڴ档

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
	����current_corner_is_90_degree()
	������struct ActionSpace current_actionspace, int corner_type
	����ֵ��int 
	���ܣ��жϵ�ǰ���ǵĶ����ռ��һ�����ǲ���90�Ƚ��������ǣ��򷵻�1���������򷵻�0��

*/
int current_corner_is_90_degree(struct ActionSpace current_actionspace, int corner_type)
{
	int x,y,i,flag,index_vertex,index_of_angle1,index_of_angle2,angle1_type,angle2_type;

	//����ǵĶ����x,y���ꡣ������������ۡ�corner_typeȡֵΪ0��1��2��3���֡�
	if(corner_type == 0)			//corner_typeֵΪ0����ʾ���Ƕ����ռ�����½�
	{
		x = current_actionspace.x;	//current_actionspace.x�Ƕ����ռ����½�x����
		y = current_actionspace.y;	//current_actionspace.y�Ƕ����ռ����½�y����
	}	
	else if(corner_type == 1)		//corner_typeֵΪ1����ʾ���Ƕ����ռ�����½�
	{
		x = current_actionspace.x + current_actionspace.width;
		y = current_actionspace.y;
	}
	else if(corner_type == 2)		//corner_typeֵΪ2����ʾ���Ƕ����ռ�����Ͻ�
	{
		x = current_actionspace.x + current_actionspace.width;
		y = current_actionspace.y + current_actionspace.height;
	}
	else							//corner_typeֵΪ3����ʾ���Ƕ����ռ�����Ͻ�
	{
		x = current_actionspace.x;
		y = current_actionspace.y + current_actionspace.height;
	}

	//��ǰ���ǵĶ����ռ������ǣ��������ѷ�����ο��ĳ�����ο飨�������ο򣩶������ڴ��������ǣ��򷵻�0��
	//˳����� beging-------------------------------------------------------------
	//flag = 0;	//flag��ǳ�ʼֵ��Ϊ0
	//forѭ������VERTEX���顣
	//for(i=0; i<pointer_to_VERTEX; i++)
	//{
	//	if(x==VERTEX[i].x && y==VERTEX[i].y)
	//	{
	//		flag = 1;
	//		index_vertex = i;	//�����VERTEX�������ҵ��ˣ�����VERTEX�����е��±����index_vertex�����С�flag�����Ϊ1��
	//		break;
	//	}
	//}

	//if(flag == 0)	//���forѭ������ʱ����û���ҵ�����flagֵ��Ȼ����0������������0��
	//	return 0;
	//˳����� end-----------------------------------------------------------------

	//�۰���� beign------------------------------------------------------
	index_vertex = half_search_vertex(VERTEX, pointer_to_VERTEX, x, y);

	if(index_vertex == -1)//�ڶ���������δ�ҵ������
		flag = 0;
	else
		flag = 1;

	if(flag == 0)	//����ڶ���������δ�ҵ�����㡣����������0��
		return 0;
	//�۰���� end---------------------------------------------------------

	//��ǰ���ǵĶ����ռ������ǣ��Ǿ��ο��һ���������ǣ��򷵻�1��
	//�����ǰ���ǵĶ����ռ��һ���������Ǿ��ο�Ľǣ����ȻΪ90�Ƚ�����
	if(x==0 && y==0 || x==BOX_WIDTH && y==0 || x==BOX_WIDTH && y==BOX_HEIGHT || x==0 && y==BOX_HEIGHT)
		return 1;

	//��û�б߾���������㣿
	//�����ĳ�����ο飨�������ο򣩵�һ���߾���������㣨�ߵĶ˵��������������ģ����㣩��������������ڴ��϶���90�Ƚ�����
	if(VERTEX[index_vertex].number_of_edge == 1)
		return 1;
	else	//û��ĳ�����ο飨�������ο򣩵�һ���߾����������
	{
		//������������ۡ�������������ڴ��ж��ٸ����ο�Ķ����غ��������ۣ���Ϊ1��2��3��4�������Ρ�
		if(VERTEX[index_vertex].number_of_angles == 1)	//ǡ��1�����ο�Ķ��㣬�϶�����90�Ƚ�����
			return 0;
		else if(VERTEX[index_vertex].number_of_angles == 2)//ǡ��2�����ο�Ķ��㣬��ʱ��Ҫ��һ�����죺����������ο����ڣ�
		{												   //����90�Ƚ�������֮������������ο鲻���ڣ����Ȼ����90�Ƚ�����
			index_of_angle1 = VERTEX[index_vertex].index_of_angles[0];
			index_of_angle2 = VERTEX[index_vertex].index_of_angles[1];

			angle1_type = ANGLE[index_of_angle1].type;
			angle2_type = ANGLE[index_of_angle2].type;

			if(angle1_type==0 && angle2_type==2 || angle1_type==1 && angle2_type==3 || angle1_type==2 && angle2_type==0 || angle1_type==3 && angle2_type==1)
				return 1;
			else
				return 0;
		}
		else if(VERTEX[index_vertex].number_of_angles == 3)	//ǡ��3�����ο�Ķ��㣬�϶���90�Ƚ�����
			return 1;
		else												//ǡ��4�����ο�Ķ��㣬�϶�����90�Ƚ�����
			return 0;
	}
	

}

/*
	����half_search_vertex()
	������struct Vertex * ptr_vertex, int number_of_vertex��int x��int y
	����ֵ��int 
	���ܣ��ڶ��������в���һ�����㡣���Ҳ������򷵻�-1���ҵ����򷵻�Ԫ���±ꡣ
	�۰���ҡ����������еĶ����Ѿ�����<y��С���ȣ���x��С���ȣ�>

*/
int half_search_vertex(struct Vertex * ptr_vertex, int number_of_vertex, int x, int y)
{
	int head,mid,tail;

	head = 0;					//2014.9.13. debug ��head��tailδ����ֵ���Ѹ����� 
	tail = number_of_vertex-1;
	while(head<=tail)
	{
		mid = (head+tail)/2;
		if(ptr_vertex[mid].x == x && ptr_vertex[mid].y == y)//���ҵ�
			return mid;
		
		if(ptr_vertex[mid].y < y || ptr_vertex[mid].y == y && ptr_vertex[mid].x < x)
			head = mid+1;
		else
			tail = mid-1;
	}

	return -1;
}

/*
	����calculate_num_of_corners_increament()
	������struct Action action, int corner_type  actionռ�����ڶ����ռ��corner_type����ǡ�
	����ֵ��int 
	���ܣ������������action�����Ժ�ʣ��ռ����������������֮ǰ�����˶��١�

*/
int calculate_num_of_corners_increament(struct Action action, int corner_type)
{
	int num_of_dismiss_corners,rec_width,rec_height,i;
	struct ActionSpace current_actionspace;
	int index_vertex,flag,index_angle,angle_type,x,y;

	//Ԥ������
	num_of_dismiss_corners = 1;//num_of_dismiss_corners��ʾ��ȥ�Ľ�������ռ�Ƕ�����ռ���Ǹ��ǿ϶���ȥ�ˡ�

	//ľ����һ�����ڵ���ľ��߶ȡ�action.width��action.height��ʾľ��Ŀ�Ⱥ͸߶ȡ�
	//rec_width��ʾľ����º���x�᷽���ϵĳ��ȡ�rec_height��ʾľ����º���y�᷽���ϵĳ��ȡ�
	if(action.pose == 0)//ľ������
	{
		rec_width = action.width;
		rec_height = action.height;
	}
	else//ľ��վ��
	{
		rec_width = action.height;
		rec_height = action.width;
	}

	//current_actionspace��ʾ�����ο�����Ժ���ռ�ݵĿռ䡣�˿ռ���������ο���ȡ�
	current_actionspace.x = action.x;
	current_actionspace.y = action.y;
	current_actionspace.width = rec_width;
	current_actionspace.height = rec_height;
	
	//���Ǽ��������οռ䣬ֻ���������ĸ��ǵĵط�������ȥ����������
	for(i=0; i<=3; i++)
	{
		if(i==corner_type)//ռ�Ƕ�����ռ���Ǹ��ǿ϶���ȥ�ˡ������ٿ���
			continue;


		//x��y�����ڿ��ǵ�������οռ��һ���ǵ����ꡣ
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

		//���������ǲ���90�Ƚ���
		if(current_corner_is_90_degree(current_actionspace,i) == 1)
		{
			num_of_dismiss_corners ++;
		}
		else
		{
			//���˴���action������i����ǣ�������һ���ǣ�270�Ƚ���������֮���ڣ�Ҳ����ȥһ���ǡ�
			//˳����ҵĳ���� begin-------------------------------------------------------
			//flag = 0;
			//for(index_vertex=0; index_vertex<pointer_to_VERTEX; index_vertex++)
			//{
			//	if(VERTEX[index_vertex].x==x && VERTEX[index_vertex].y==y)
			//	{
			//		flag = 1;
			//		break;
			//	}
			//}
			//˳����ҵĳ���� end----------------------------------------------------------

			//�۰����
			index_vertex = half_search_vertex(VERTEX, pointer_to_VERTEX, x, y);

			if(index_vertex == -1)//�ڶ���������δ�ҵ������
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
	//	printf("����������\n");
	//}
	return 4-2*num_of_dismiss_corners;
}


/*
	����calculate_num_of_lines()
	������struct Action action
	����ֵ��double 
	���ܣ�������������붯���ռ����������

*/
int calculate_num_of_lines(struct Action action, int index_actionspace)
{
	struct ActionSpace actionspace;
	int rec_width,rec_height;


	actionspace = ACTION_SPACE[index_actionspace];

	if(action.pose == 0)//0��ʾ���š�1��ʾվ�š�
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
	����current_action_is_better()
	������struct Action current_action, struct Action best_action
	����ֵ��int 
	���ܣ��жϣ���current_action��best_action���ţ��򷵻�1�����򷵻�0��

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
	����basic_algorithm_choose_an_action()
	��������
	����ֵ������-1��ʾʧ�ܣ��޶�������������0��ʾѡ����һ��������
	���ܣ������㷨B0��ѡ��һ������
*/
int basic_algorithm_choose_an_action(struct Action * ptr_action, int * ptr_index_actionspace)
{
	int index_actionspace,index_rectangle,current_pose,rec_width,rec_height,corner_type,first_action_flag,first_rec_flag;
	struct ActionSpace current_actionspace;
	struct RectangleInfo current_rectangle,rectangle;
	struct Action current_action,best_action;
	int current_actionspace_valid_flag;

	//�����ǰ�����ռ������е�ÿ�������ռ��ÿ�����Ƿ�Ϊ90�Ƚ���
	//pointer_to_ACTION_SPACE��ȫ�ֱ�������ֵΪ��ǰ�����ռ������д洢�Ķ����ռ�����
	//index_actionspaceΪѭ�����Ʊ���
	for(index_actionspace=0; index_actionspace<pointer_to_ACTION_SPACE; index_actionspace++)
	{
		current_actionspace = ACTION_SPACE[index_actionspace];
		if(current_actionspace.flag == 0)//��Ч�Ķ����ռ�
		{
			continue;
		}

		//���ο��Ǳ������ռ���ĸ��ǡ�����current_corner_is_90_degree�����������㡣
		for(corner_type=0; corner_type<=3; corner_type++)
		{
			//�����ռ�ACTION_SPACE[index_actionspace]��corner_90_degree�ֶ���һ����Ԫ�ص�int�����飬
			//Ԫ�ص�ֵ��ʾ���ǲ���90�Ƚ�����1Ϊ�ǣ�0Ϊ��
			ACTION_SPACE[index_actionspace].corner_90_degree[corner_type] = 
				current_corner_is_90_degree(current_actionspace, corner_type);
		}

	}

	first_action_flag = 1;//first_action_flag��ǳ�ʼ��Ϊ1����ʾ��һ�������ǵĺ���ռ�Ƕ�����
	for(index_actionspace=0; index_actionspace<pointer_to_ACTION_SPACE; index_actionspace++)
	{
		//ѭ��������ÿһ�������ռ�
		current_actionspace = ACTION_SPACE[index_actionspace];
		if(current_actionspace.flag == 0)//��Ч�Ķ����ռ䲻����
		{
			continue;
		}
		
		//�����ǰ�����ռ䣬���о��ο鶼�Ų��£�����current_actionspace_valid_flag���ֵΪ0��
		//��current_actionspace_valid_flag�������ó�ʼֵΪ0��
		current_actionspace_valid_flag = 0;
		
		first_rec_flag = 1;//��ͬ��״��С�ľ��ο�ֻ����һ�������first_rec_flag���ʼֵΪ1
		for(index_rectangle=0; index_rectangle<NUM_OF_RECTANGLES; index_rectangle++)
		{
			current_rectangle = RECTANGLES_INFO[index_rectangle];
			if(current_rectangle.flag == 1)	
				continue;	//ľ���Ѿ��ں��ڣ�������

			if(first_rec_flag == 1)	//��first_rec_flagֵΪ1�����Ϊ0�������ο�current_rectangle��¼��rectangle�����С�
			{
				first_rec_flag = 0;
				rectangle = current_rectangle;
			}
			else	//��first_rec_flagֵΪ0���򿴱����ο�current_rectangle�������rectangle��ȫ��ͬ����������ο鲻���ǡ�
			{
				if(current_rectangle.width == rectangle.width && current_rectangle.height == rectangle.height)
					continue;
				else//�����ο�current_rectangle�������rectangle����ȫ��ͬ����������ο�Ҫ���ǡ�
				{
					rectangle = current_rectangle;
				}
			}

			current_action.width = current_rectangle.width;		//���ο�ĳ��߳�
			current_action.height = current_rectangle.height;	//���ο�Ķ̱߳�

			for(current_pose=0; current_pose<=1; current_pose++)//0��ʾľ�����ţ�1��ʾվ��
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
				//rec_width��ʾ�����ο鰴��current_pose������̬���º���x�᷽���ϵĳ��ȡ�
				//rec_height��ʾ�����ο鰴��current_pose������̬���º���y�᷽���ϵĳ��ȡ�
	
				if(rec_width>current_actionspace.width || rec_height>current_actionspace.height)
					continue;	//���ο鰴��current_pose������̬�ڱ������ռ��зŲ��£��򲻿��Ǵ˶�����

				//�϶��ܷ��£���current_actionspace_valid_flag�����Ϊ1����ʾ�������ռ�϶���ĳ�����ο��ܷŽ�ȥ��
				current_actionspace_valid_flag = 1;
				//�������ο��Ƕ����ռ���ĸ��ǡ������ĸ�ռ�Ƕ�����
				for(corner_type=0; corner_type<=3; corner_type++)
				{
					//current_action�ǵ�ǰ���ǵ�ռ�Ƕ�����current_action.index�ֶα�ʾ���ž��ο���±ꡣ
					current_action.index = index_rectangle;
					//current_action.pose��ʾ��������̬
					current_action.pose = current_pose;

					//current_action.x��current_action.y��ʾ���ռ�Ƕ����������ο���õ�λ�á����ο�����½ǵ�x��y���ꡣ
					if(corner_type==0)			//ռ�����ռ�����½�
					{
						current_action.x = current_actionspace.x;
						current_action.y = current_actionspace.y;
					}
					else if(corner_type==1)		//ռ�����ռ�����½�
					{
						current_action.x = current_actionspace.x + current_actionspace.width - rec_width;
						current_action.y = current_actionspace.y;
					}
					else if(corner_type==2)		//ռ�����ռ�����Ͻ�
					{
						current_action.x = current_actionspace.x + current_actionspace.width - rec_width;
						current_action.y = current_actionspace.y + current_actionspace.height - rec_height;
					}
					else						//ռ�����ռ�����Ͻ�
					{
						current_action.x = current_actionspace.x;
						current_action.y = current_actionspace.y + current_actionspace.height - rec_height;
					}

					//�����ǰ�ǣ���corner_type����������90�Ƚ������򲻿��Ǵ˶�����
					if(ACTION_SPACE[index_actionspace].corner_90_degree[corner_type] == 0)
						continue;

					//����current_action�ĸ���ָ��
					//����current_action�����Ժ󣬽��������Ӷ��١�
					current_action.num_of_corners_increament = calculate_num_of_corners_increament(current_action, corner_type);
					//����current_action�����������ڶ����ռ��������
					current_action.num_of_lines = calculate_num_of_lines(current_action, index_actionspace);

					//2014.9.19. debug��current_action�� index_actionspace�ֶ�û��д���Ѹ�����
					current_action.index_actionspace = index_actionspace;					
					
					//first_action_flag���Ϊ1����ʾ��һ�������ǵĺ���ռ�Ƕ�����
					if(first_action_flag == 1)
					{
						//��first_action_flag�������Ϊ0
						first_action_flag = 0;
						//����ǰ������Ϊbest_action
						best_action = current_action;
						//��¼��ǰ�������ڵĶ����ռ��ڶ����ռ������е��±�
						* ptr_index_actionspace = index_actionspace;
					}
					else
					{
						//���ǵ�һ�������ǵĺ���ռ�Ƕ���
						//���current_action����best_action���򽫵�ǰ������Ϊbest_action��
						if(current_action_is_better(current_action, best_action) == 1)
						{
							best_action = current_action;
							//��¼��ǰ�������ڵĶ����ռ��ڶ����ռ������е��±�
							* ptr_index_actionspace = index_actionspace;
						}
					}


				}
			}
		}

		//���current_actionspace�ı��ֵΪ0����ʾ��������ռ�װ�����κξ��ο顣������Ϊ��Ч��
		if(current_actionspace_valid_flag == 0)
			ACTION_SPACE[index_actionspace].flag = 0;
	}

	
	if(first_action_flag == 1)
	{
		//��ѭ������ʱ����first_action_flagֵ��Ϊ1����ʾ��ǰ����£�û�к���ռ�Ƕ�����������������-1��
		return -1;
	}
	else
	{
		//����best_actionд��ָ��ptr_actionָ����ڴ浥Ԫ���ṹ�壩����������0��
		*ptr_action = best_action;
		return 0;		
	}
}

/*
	����do_an_action()
	������struct Action action, int index_actionspace
	����ֵ��void
	���ܣ���һ������������������ݽṹ��
*/
void do_an_action(struct Action action, int index_actionspace)
{
	int index_rectangle,index_angle,i,index_edge,corner_type,x,y,flag,index_vertex,number_of_angles;
	int rectangle_x,rectangle_y,rectangle_pose,rectangle_width,rectangle_height,rec_width,rec_height;
	struct ActionSpace actionspace,new_actionspace1,new_actionspace2,new_actionspace3,new_actionspace4;
	int overlapx,overlapy,overlap;
	int x1,y1,x2,y2;
	struct Vertex temp_vertex;

	//����������ݽṹ

	//1. �������� ok
	index_rectangle = action.index;
	RECTANGLES_INFO[index_rectangle].x = action.x;
	RECTANGLES_INFO[index_rectangle].y = action.y;
	RECTANGLES_INFO[index_rectangle].flag = 1;
	RECTANGLES_INFO[index_rectangle].pose = action.pose;
	
	//2. ������ ok
	index_angle = pointer_to_ANGLE;
	for(i=0; i<=3; i++)
	{
		ANGLE[index_angle+i].index = action.index;
		ANGLE[index_angle+i].type = i;
	}
	pointer_to_ANGLE += 4;

	//3. ������  ok
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
	//�ߵĶ˵�����(x1,y1)��(x2,y2)��x1<=x2, y1<=y2��
	EDGE[index_edge+0].x1 = action.x;				EDGE[index_edge+0].y1 = action.y;
	EDGE[index_edge+0].x2 = action.x+rec_width;		EDGE[index_edge+0].y2 = action.y;

	EDGE[index_edge+1].x1 = action.x+rec_width;		EDGE[index_edge+1].y1 = action.y;
	EDGE[index_edge+1].x2 = action.x+rec_width;		EDGE[index_edge+1].y2 = action.y+rec_height;

	EDGE[index_edge+2].x1 = action.x;				EDGE[index_edge+2].y1 = action.y+rec_height;
	EDGE[index_edge+2].x2 = action.x+rec_width;		EDGE[index_edge+2].y2 = action.y+rec_height;

	EDGE[index_edge+3].x1 = action.x;				EDGE[index_edge+3].y1 = action.y;
	EDGE[index_edge+3].x2 = action.x;				EDGE[index_edge+3].y2 = action.y+rec_height;

	pointer_to_EDGE += 4;

	//4. �������� 
	//4.1 ��������Ӧ���ο���ĸ����㡣
	for(corner_type=0; corner_type<=3; corner_type++)
	{
		//���ο��Ǳ�������Ӧ���ο���ĸ����㡣
		//������������꣬�����������
		if(corner_type==0)
		{
			x = action.x;
			y = action.y;
		}
		else if(corner_type==1)
		{
			x = action.x + rec_width; //2014.5.15 debug ����������£� rec_width,����action.width
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

		//�������Ƿ���VERTEX�����С�
		//˳����ҳ���� begin---------------------------------------------------------------
		//flag = 0;
		//for(index_vertex=0; index_vertex<pointer_to_VERTEX; index_vertex++)
		//{
		//	if(x==VERTEX[index_vertex].x && y==VERTEX[index_vertex].y)
		//	{
		//		flag = 1; 
		//		break;
		//	}
		//}
		//˳����ҳ���� end-----------------------------------------------------------------
		//�۰���ҵĳ����begin---------------------------------------------------------------
		index_vertex = half_search_vertex(VERTEX, pointer_to_VERTEX, x, y);
		if(index_vertex == -1)	//����������δ�ҵ��������¶���
			flag = 0;
		else
			flag = 1;
		//�۰���ҵĳ����end---------------------------------------------------------------

		//flagΪ1���϶��㡣0���¶��㡣

		//����VERTEX�����У����϶�������һ���ǡ��϶��㲻��Ҫ�����µıߡ�
		if(flag == 1)
		{	
			number_of_angles = VERTEX[index_vertex].number_of_angles;
			VERTEX[index_vertex].index_of_angles[number_of_angles] = pointer_to_ANGLE - (4-corner_type);
			VERTEX[index_vertex].number_of_angles ++;
		}
		else //������VERTEX�����У����¶���ǡ��һ���ǡ�
		{
			index_vertex = pointer_to_VERTEX;
			VERTEX[index_vertex].x = x;
			VERTEX[index_vertex].y = y;
			VERTEX[index_vertex].number_of_angles = 1;
			VERTEX[index_vertex].index_of_angles[0] = pointer_to_ANGLE - (4-corner_type);
			pointer_to_VERTEX ++;
			//����
			while(index_vertex >= 1)
			{
				if(VERTEX[index_vertex-1].y < VERTEX[index_vertex].y ||
					VERTEX[index_vertex-1].y == VERTEX[index_vertex].y && VERTEX[index_vertex-1].x < VERTEX[index_vertex].x)
					break;
				else
				{
					//����
					temp_vertex = VERTEX[index_vertex-1];
					VERTEX[index_vertex-1] = VERTEX[index_vertex];
					VERTEX[index_vertex] = temp_vertex;

					index_vertex --;
				}
			}


			//��Ҫ�����¶����Ƿ�סһ���ߡ�
			//2014.9.20. debug: VERTEX[index_vertex].number_of_edgeӦ�ø���ֵΪ0. 
			VERTEX[index_vertex].number_of_edge = 0;	
			
			//�������ǿ���������ߡ�
			//�±ߣ�
			if(y==0 && x>0 && x<BOX_WIDTH)
			{
				VERTEX[index_vertex].number_of_edge = 1;
				continue;
			}
			//�ұ�
			if(x==BOX_WIDTH && y>0 && y<BOX_HEIGHT)
			{
				VERTEX[index_vertex].number_of_edge = 1;
				continue;
			}
			//�ϱ�
			if(y==BOX_HEIGHT && x>0 && x<BOX_WIDTH)
			{
				VERTEX[index_vertex].number_of_edge = 1;
				continue;
			}
			//���
			if(x==0 && y>0 && y<BOX_HEIGHT)
			{
				VERTEX[index_vertex].number_of_edge = 1;
				continue;
			}

			//�����������ǿ��������ں����еľ��ε�4���ߡ�����ͬ����Ĵ������ơ� 
			//2014.9.21. debug: index_edgeֵΪ0-3��4�����Ǿ��ο�ıߣ���indexֵΪ-1����-1���±�ȥRECTANGLES_INFO��������Ԫ�أ����ܻ����������
			//ǰ��������ʱȷʵ������������
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
			
				//�±ߣ�
				if(y==rectangle_y && x>rectangle_x && x<rectangle_x+rectangle_width)
				{
					VERTEX[index_vertex].number_of_edge = 1;
					break;
				}
				//�ұ�
				if(x==rectangle_x+rectangle_width && y>rectangle_y && y<rectangle_y+rectangle_height)
				{
					VERTEX[index_vertex].number_of_edge = 1;
					break;
				}
				//�ϱ�
				if(y==rectangle_y+rectangle_height && x>rectangle_x && x<rectangle_x+rectangle_width)
				{
					VERTEX[index_vertex].number_of_edge = 1;
					break;
				}
				//���
				if(x==rectangle_x && y>rectangle_y && y<rectangle_y+rectangle_height)
				{
					VERTEX[index_vertex].number_of_edge = 1;
					break;
				}

			}
		}
		
	}

	//4.2 2014.9.14. debug��Ӧ�ÿ��ǡ����������е���Щ���㣬�Ƿ�ס��������Ӧ���ο�������ߡ�������ǡ��λ�ڱߵĶ˵�ģ����㡣��
	for(index_edge=pointer_to_EDGE-4; index_edge<=pointer_to_EDGE-1; index_edge++)
	{
		x1 = EDGE[index_edge].x1;	y1 = EDGE[index_edge].y1;
		x2 = EDGE[index_edge].x2;	y2 = EDGE[index_edge].y2;

		for(index_vertex=0; index_vertex<=pointer_to_VERTEX; index_vertex++)
		{
			//�����㱻�߶�ס�����������
			if(index_edge %2 ==0)	//���
			{
				if(VERTEX[index_vertex].y == y1 && VERTEX[index_vertex].x > x1  && VERTEX[index_vertex].x < x2)
				{
					VERTEX[index_vertex].number_of_edge = 1;
				}
			}
			else					//����
			{
				if(VERTEX[index_vertex].x == x1 && VERTEX[index_vertex].y > y1  && VERTEX[index_vertex].y < y2)
				{
					VERTEX[index_vertex].number_of_edge = 1;
				}				
			}
		}
	}

	//5. �����ռ�  
 
	//5.1 ���������ڵĶ����ռ䡣
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
	if(action.y == actionspace.y)//����ռ�����ռ�������ǡ�
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
	if(action.x == actionspace.x)//����ռ�����ռ�������ǡ�
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

	//���¶����ռ����飬������������ۡ�
	if(new_actionspace1.height>0 && new_actionspace2.width>0)
	{	
		ACTION_SPACE[index_actionspace] = new_actionspace1;//Ӧ��дnew_actionspace1��undo�ֶ�
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
		//ɾȥ�˶����ռ�
		ACTION_SPACE[index_actionspace].flag = 0;
	}

	//5.2 ���������Ƿ��붯���ռ����������������ռ��ص��������ص�������Ҫ���¡�
	for(i=0; i<pointer_to_ACTION_SPACE; i++)
	{
		actionspace = ACTION_SPACE[i];
		//�ж�action��actionspace�Ƿ��ص���
		if(action.x>=actionspace.x+actionspace.width || actionspace.x>=action.x+rec_width)//���ص�
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

		if(overlap == 0)//���ص�������Ҫ���Ǹ��¶����ռ䡣
			continue;


		//������θ��¶����ռ䡣
		//���˶�����ȫ���Ƕ����ռ䣬��ɾȥ�˶����ռ䡣
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
			//�оٳ��ĸ��µĶ����ռ�
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

			//д�붯���ռ�����
			ACTION_SPACE[i] = new_actionspace1;
			ACTION_SPACE[pointer_to_ACTION_SPACE ++] = new_actionspace2;
			ACTION_SPACE[pointer_to_ACTION_SPACE ++] = new_actionspace3;
			ACTION_SPACE[pointer_to_ACTION_SPACE ++] = new_actionspace4;
		}
		else if( (action.x<=actionspace.x || action.x+rec_width>=actionspace.x+actionspace.width) &&
			action.y>actionspace.y && action.y+rec_height<actionspace.y+actionspace.height)
		{
			//�оٳ������������µĶ����ռ�
			
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
				//д�붯���ռ�����
				
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

				//д�붯���ռ�����
				

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

				//д�붯���ռ�����
				

				ACTION_SPACE[i] = new_actionspace1;
				ACTION_SPACE[pointer_to_ACTION_SPACE ++] = new_actionspace2;
				ACTION_SPACE[pointer_to_ACTION_SPACE ++] = new_actionspace3;
			}
		}
		else if(action.x>actionspace.x && action.x+rec_width<actionspace.x+actionspace.width &&
			(action.y<=actionspace.y || action.y+rec_height>=actionspace.y+actionspace.height) )
		{
			//�оٳ������������µĶ����ռ�
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
				//д�붯���ռ�����
				

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

				//д�붯���ռ�����
				

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

				//д�붯���ռ�����
				

				ACTION_SPACE[i] = new_actionspace1;
				ACTION_SPACE[pointer_to_ACTION_SPACE ++] = new_actionspace3;
				ACTION_SPACE[pointer_to_ACTION_SPACE ++] = new_actionspace4;
			}
		}
		else //(action.x<=actionspace.x || action.x+rec_width>=actionspace.x+actionspace.width) && (action.y<=actionspace.y || action.y+rec_height>=actionspace.y+actionspace.height)	
		{
			if(action.x<=actionspace.x && action.x+rec_width>=actionspace.x+actionspace.width)
			{
				//ֻ��1���µĶ����ռ�
				if(action.y<=actionspace.y && action.y+rec_height<actionspace.y+actionspace.height)
				{
					new_actionspace1.width = actionspace.width;
					new_actionspace1.height = (actionspace.y+actionspace.height) - (action.y+rec_height);
					new_actionspace1.x = actionspace.x;
					new_actionspace1.y = action.y + rec_height;
					new_actionspace1.flag = 1;
					//���¶����ռ�����
					

					ACTION_SPACE[i] = new_actionspace1;
				}
				else//action.y>actionspace.y && action.y+rec_height>=actionspace.y+actionspace.height
				{
					new_actionspace1.width = actionspace.width;
					new_actionspace1.height = action.y - actionspace.y;
					new_actionspace1.x = actionspace.x;
					new_actionspace1.y = actionspace.y;
					new_actionspace1.flag = 1;
					//���¶����ռ�����
					

					ACTION_SPACE[i] = new_actionspace1;
				}
			}
			else if(action.y<=actionspace.y && action.y+rec_height>=actionspace.y+actionspace.height)
			{
				//ֻ��1���µĶ����ռ�
				if(action.x<=actionspace.x && action.x+rec_width<actionspace.x+actionspace.width)
				{
					new_actionspace3.width = (actionspace.x+actionspace.width) - (action.x+rec_width);
					new_actionspace3.height = actionspace.height;
					new_actionspace3.x = action.x + rec_width;
					new_actionspace3.y = actionspace.y;
					new_actionspace3.flag = 1;
					//���¶����ռ�����
					

					ACTION_SPACE[i] = new_actionspace3;
				}
				else//action.x>actionspace.x && action.x+rec_width>=actionspace.x+actionspace.width
				{
					new_actionspace3.width = action.x - actionspace.x;
					new_actionspace3.height = actionspace.height;
					new_actionspace3.x = actionspace.x;
					new_actionspace3.y = actionspace.y;
					new_actionspace3.flag = 1;
					//���¶����ռ�����
					

					ACTION_SPACE[i] = new_actionspace3;
				}
			}
			else
			{
				//�оٳ������µĶ����ռ�
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

				//д�붯���ռ�����
				

				ACTION_SPACE[i] = new_actionspace1;
				ACTION_SPACE[pointer_to_ACTION_SPACE ++] = new_actionspace3;
			}
		}


	}

	
	
}


/*
	����delete_invailid_and_reluctant_action_space()
	������void
	����ֵ��void
	���ܣ�ɾȥ��Ч�Ķ����ռ䡣ɾȥ����Ķ����ռ䣬�еĶ����ռ䱻���������ռ��������Ϊ���࣬ɾȥ��
*/
void delete_invailid_and_reluctant_action_space()
{
	int i,j,contian_flag;


	//ɾȥ��Ч�Ķ����ռ䡣flagֵΪ0�Ķ����ռ�����Ч�ġ�
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


	//�����а�����ϵ�Ķ����ռ�
	i = 0;
	while(1)
	{
		if(i >= pointer_to_ACTION_SPACE)
			break;

		//��i�Ƿ����������ռ���������ǣ���contain_flag=1��������contain_flag=0.
		contian_flag = 0;
		for(j=0; j<pointer_to_ACTION_SPACE; j++)
		{
			if(j == i)
				continue;
			if(ACTION_SPACE[j].flag == 0)
				continue;
			//��j����i����contian_flag=1. break;
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
			//ɾȥi
			//for(j=i; j<pointer_to_ACTION_SPACE-1; j++)
			//	ACTION_SPACE[j] = ACTION_SPACE[j+1];
			//pointer_to_ACTION_SPACE--;
			ACTION_SPACE[i].flag = 0;
			i++;
		}
	}

	//ɾȥ��Ч�Ķ����ռ䡣flagֵΪ0�Ķ����ռ�����Ч�ġ�
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

	
	//1. ��������
	//���������ݴ��ļ�����      ���Ӹ߶�ֵ���ļ����룬����c1.txt��˵�����ӿ��ֵ��Ϊ20.

	//���㵱ǰ���̵�c�ļ����е�c1.txt�������
	strcpy(file_name, "c\\c1.txt");  //���Ҫ�������������������޸���һ�С������Ϊ��c\\c4.txt��
	fp = fopen(file_name, "r");
	if(fp==NULL)
	{
		printf("Can not open the file.\n");
		return -1;
	}

	build_an_instance(fp);

	printf("ľ����=%d\n", NUM_OF_RECTANGLES);
	//printf("���ӿ��=%d\n", BOX_WIDTH);
	printf("���Ӹ߶�=%d\n", BOX_HEIGHT);
	//for(i=0; i<NUM_OF_RECTANGLES; i++)
	//{
	//	printf("%d %d %d\n", i, WIDTHS_OF_RECTANGLES[i], HEIGHTS_OF_RECTANGLES[i]);
	//}

	fclose(fp);

	start = clock(); 

	//2. ����
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

		//������
		sum_of_area = 0;
		for(i=0; i<NUM_OF_RECTANGLES; i++)
		{
			if(RECTANGLES_INFO[i].flag == 1)
				sum_of_area += RECTANGLES_INFO[i].width*RECTANGLES_INFO[i].height;
		}

		ratio = (double)sum_of_area / (double)(BOX_WIDTH*BOX_HEIGHT);

		end = clock(); 

		printf("�������=%d\n", sum_of_area);

		printf("������Ϊ��%.2f\n", 100*ratio);

		ratio = (double)sum_of_area / (double)(SUM_OF_REC_AREA);

		printf("ľ�������=%d\n",SUM_OF_REC_AREA);

		printf("������ӵ�ľ�������=%d\n",sum_of_area);

		printf("ľ�������Ϊ��%.2f\n", 100*ratio);
	
		printf("����ʱ��Ϊ��%f\n", (double)(end - start) / CLK_TCK);
	
		if(sum_of_area == SUM_OF_REC_AREA)
		{
			printf("ľ��ȫ���������\n");
			break;
		}
		BOX_WIDTH += 1;
	}
	//return 0;

	//getchar();

	//��ʾ��ֹ��ֵ�ʾ��ͼ
	dis = 30;
	initgraph(1240, 640);

	
	//��ʾ���ο�
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
	//3. �ͷ��ڴ棬������
	free_mem();


	return 0;
}

