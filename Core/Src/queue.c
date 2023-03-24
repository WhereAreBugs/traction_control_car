#include "Queue.h"
#include <stdio.h>
#include "stdlib.h"
void QueueInit(Queue* pq)
{
    pq->head = pq->tail = NULL;

}
void QueueDestory(Queue* pq)
{
    QNode* cur = pq->head;
    while (cur)
    {
        QNode* del = cur;
        cur = cur->next;
        free(del);
        del = NULL;

    }

    pq->head = pq->tail = NULL;


}

void QueuePush(Queue* pq, QueueDataType x)
{
    QNode* newnode = (QNode*)malloc(sizeof(QNode));
    if (newnode == NULL)
    {
        perror("malloc fail!!!");
        exit(-1);
    }
    else
    {
        newnode->data = x;
        newnode->next = NULL;

    }

    //如果队列为空
    if (pq->head ==NULL&& pq->tail==NULL)//说明队列空
    {
        pq->head = pq->tail = newnode;
    }
    else
    {
        pq->tail->next = newnode;
        pq->tail = pq->tail->next;
    }
}


float QueueVariance(Queue* pq)
//方差
{
    float sum = 0;
    float sum2 = 0;
    float mean = 0;
    float mean2 = 0;
    float variance = 0;
    int n = 0;
    QNode* cur = pq->head;
    while (cur)
    {
        sum += cur->data;
        sum2 += cur->data*cur->data;
        ++n;
        cur = cur->next;
    }
    mean = sum / n;
    mean2 = sum2 / n;
    variance = mean2 - mean*mean;
    return variance;
}
void QueuePop(Queue* pq)//队列特性  头删
{
    //一个节点
    if (pq->head->next == NULL)
    {
        free(pq->head);
        pq->head = pq->tail = NULL;
    }

    else//很多节点
    {
        QNode* del = pq->head;
        pq->head = pq->head->next;
        free(del);
        del = NULL;
    }




}

QueueDataType QueueFront(Queue* pq)
{
    return pq->head->data;
}


QueueDataType QueueBack(Queue* pq)
{


    return pq->tail->data;
}
QNode * c_head_ptr(Queue* pq)
{
    return pq->head;
}
_Bool QueueEmpty(Queue* pq)
{
    return pq->head == NULL;   //空返回1  有元素0

}
float QueueGet(Queue *pq,int index)
{
    QNode* cur = pq->head;
    int i = 0;
    while (cur)
    {
        if (i == index)
        {
            return cur->data;
        }
        cur = cur->next;
        ++i;
    }
    return -999;//表示没有找到
}
int QueueSize(Queue* pq)
{


    int n = 0;
    QNode* cur = pq->head;
    while (cur)
    {
        ++n;
        cur = cur->next;
    }

    return n;


}