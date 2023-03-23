//
// Created by 神奇bug在哪里 on 3/23/23.
//

#ifndef SMART_QUEUE_H
#define SMART_QUEUE_H

typedef float QueueDataType;

typedef struct QueueNode
{
    struct QueueNode* next;
    QueueDataType data;

}QNode;

typedef struct Queue
{

    QNode* head;
    QNode* tail;


}Queue;
void QueueInit(Queue* pq);   //初始化队列
void QueueDestory(Queue* pq);    //销毁队列

void QueuePush(Queue* pq, QueueDataType x); //插入元素
void QueuePop(Queue* pq);   //出队列
float QueueGet(Queue *pq,int index); //获取队列中的元素
QueueDataType QueueFront(Queue* pq);  //查看队头元素
QueueDataType QueueBack(Queue* pq);   //查看队尾元素
QNode * c_head_ptr(Queue* pq); //返回队列头指针

_Bool QueueEmpty(Queue* pq);  //检查队列是否为空
int QueueSize(Queue* pq);    //查看队列的长度
#endif //SMART_QUEUE_H
