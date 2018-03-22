#include <stdio.h>
#include <stdlib.h>
void sender();
void receiver();
int i,slot;
int ack=0,flag=0,pno=0;
int main()
{
    printf("Enter the time slot:\n");
    scanf("%d",&slot);
   for(i=0;i<9;i++)
   {
      printf("Transfer no. %d \t",i+1);
      sender();
      receiver();
   }

    return 0;
}
void sender()
{

    if(ack==0)
    {
        printf("Sending packet no.%d \t",pno+1);

        flag++;
        ack++;

    }
    else{
       printf("**Resending packet** \t");

    }
}
void receiver()
{
    if(flag==1)
    {

        if(i+1==3 || i+1==6 ||i+1==7)
        {
            printf("**Packet corrupted** \n");

        }
        else{
        printf("Received packet no.%d \n",pno+1);
        ack--;
        flag--;
        pno++;
        }
    }
}
