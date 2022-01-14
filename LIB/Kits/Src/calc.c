#include "stdint.h"
#include "calc.h"
//#include "global_var.h"
#include "global.h"
#include "sys_conf.h"
//uint8_t checksum_u8(uint8_t* data_ptr, uint16_t data_num)
//{
//		uint16_t 	i;
//		uint8_t 	checksum_res;
//		checksum_res = *data_ptr;
//		for(i=1;i<data_num;i++)
//		{
//				checksum_res ^= *(data_ptr+i);
//		}
//		return checksum_res;
//}

//uint16_t checksum_u16(uint16_t* data_ptr, uint16_t data_num)
//{
//		uint16_t 	i;
//		uint16_t 	checksum_res;
//		checksum_res = *data_ptr;
//		for(i=1;i<data_num;i++)
//		{
//				checksum_res ^= *(data_ptr+i);
//		}
//		return checksum_res;
//}

//uint8_t xor_checksum(uint8_t* data_ptr, uint16_t data_num)
//{
//		uint16_t i;
//		uint8_t 	checksum_res;
//		checksum_res = 0;
//		for(i=0;i<data_num;i++)
//		{
//				checksum_res ^= *(data_ptr+i);
//		}
//		return checksum_res;
//}

int16_t lim_min_max(int16_t min, int16_t max, int16_t data)
{
		if(data <= min)
		{
				return min;
		}
		else if(data >= max)
		{
				return max;
		}
		else
		{
				return data;
		}
}

int16_t bin_search(uint16_t *a_ptr, uint16_t array_size, uint16_t key)  
{     
    int low = 0, high = array_size - 1, mid;  
		
		if((key < *(a_ptr))||(key > *(a_ptr+array_size)))
		{
				return -1;
		}	
      
    while (low <= high)  
    {         
        mid = (low + high) / 2; 
          
        if ((*(a_ptr+mid) <= key)&&(*(a_ptr+mid+1) > key)&&((mid+1)<array_size))
				{
            return mid;   
				}
				else
				{
						if (*(a_ptr+mid) > key)      
						{
								high = mid - 1;
						}
						else
						{
								low = mid + 1;
						}
				}
    }
    return -1;    
}

void quick(uint16_t *a,int16_t i,int16_t j)        //��������
{
    int16_t m,n,temp;
    uint16_t k;
    m=i;
    n=j;
    k=a[(i+j)/2]; /*ѡȡ�Ĳ���*/
    do
    {
        while(a[m]<k&&m<j)
        {
            m++;    /* �������ұ�k���Ԫ��*/
        }
        while(a[n]>k&&n>i)
        {
            n--;    /* ���ҵ����ұ�kС��Ԫ��*/
        }
        if(m<=n)   /*���ҵ��������������򽻻�*/
        {
            temp=a[m];
            a[m]=a[n];
            a[n]=temp;
            m++;
            n--;
        }
    }
    while(m<=n);
    if(m<j)
    {
        quick(a,m,j);    /*���õݹ�*/
    }
    if(n>i)
    {
        quick(a,i,n);
    }
}


/**
*  ��λֵƽ���˲�
* @param pData:û���˲������� 
* @param nSize:���ݴ�С 
* @return:�˲���ֵ
*/
unsigned short MedianFilter(unsigned short* pData,int nSize)
{
    unsigned short max,min;
    int sum;
    int ave;
    int i;
    if(nSize>2)
    {
        max = pData[0];
        min = max;
        sum = 0;
        for(i=0;i<nSize;i++)
        {
            sum += pData[i];            
            if(pData[i]>max)
            {
                max = pData[i];   //һ��ѭ��֮��max��������ֵ
            }

            if(pData[i]<min)
            {
                min = pData[i];   //һ��ѭ��֮��min������С��ֵ
            }
//						rt_kprintf("pData[i] = %d\n", pData[i] );	
        }

        sum = sum-max-min;       //ȥ������ֵ����С��ֵ
				ave=sum/(nSize-2);
//				rt_kprintf("sum = %d,ave = %d,u16Ain[i] = %d\n", sum,ave );					
        return ave; //��N-2������ƽ��ֵ          
    }

    return 0;
}
//ð�ݷ�
void bubble_sort(int16_t *a,uint8_t n)
{
	uint8_t i,j;
	for(i = 0;i<n;i++)
	{
		for(j = i;j < n;j++)
		{
			if(*(a+i)>*(a+j))
			{
				int temp;
				temp = *(a+i);
				*(a+i) = *(a+j);
				*(a+j) = temp;
			}
		}
	}
}





