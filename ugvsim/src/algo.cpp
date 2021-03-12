#include<stdio.h>
#include<math.h>

class ugv
{
  public:
    struct xy
  {
    float x;
    float y;  
  };
  typedef struct xy xy;

  xy dest;
  ugv(float x, float y)
  {
    dest.x=x;
    dest.y=y;
  }

  float direct(xy inital, xy current)
  {
      printf("------------------------------\n");
      printf("Before transformation:\n");
      printf("Destination:\n");
      printf("      X= %f\n",dest.x);
      printf("      Y= %f\n",dest.y);
      printf("Initial:\n");
      printf("      X= %f\n",inital.x);
      printf("      Y= %f\n",inital.y);
      printf("Current:\n");
      printf("      X= %f\n",current.x);
      printf("      Y= %f\n",current.y);

      double temp_x=dest.x-inital.x;
      double temp_y=dest.y-inital.y;

      current.x=current.x-inital.x;
      current.y=current.y-inital.y;

      double x=atan2(temp_y,temp_x);
      double y=-current.x*sin(x)+current.y*cos(x);

      printf("After transformation:\n");
      printf("      X= %f\n",dest.x);
      printf("      Y= %f\n",dest.y);
      printf("Initial:\n");
      printf("      X= %f\n",inital.x);
      printf("      Y= %f\n",inital.y);
      printf("Current:\n");
      printf("      X= %f\n",current.x);
      printf("      Y= %f\n",current.y);
      printf("[+]Current Y= %lf \n",y);
      printf("[+]Destination: X= %lf  Y= %lf\n",(dest.x*cos(x)+dest.y*sin(x)), (-dest.x*sin(x)+dest.y*cos(x)));

      if (y<0)   return (rand()/pow(10,9));
      else   return (-rand()/pow(10,9));
  }

};
/*
**************
**** Test ****
**************
* These are some test cases fo the algorithm
* Expected results:
  (random no.)
  -(random no.)
  -(random no.)
  (random no.)
*/
int main(void)
{
  ugv algo[4]={ugv(30,33), ugv(30,25), ugv(27,25), ugv(27,33)};
  ugv::xy initial, current;

  initial.x=28;
  initial.y=29;

  current.x=30;
  current.y=30;

  printf("For different positions of destinations\n");
  for (int i=0;i<4;i++)
    printf("%f\n",algo[i].direct(initial,current));

  printf("test of the random output\n");
  for(int i=0;i<4;i++)
  {
    ugv random(30,33);
    printf("case %d\n   %f\n",i,random.direct(initial,current));
  }
  printf("test complited\n");
  return 0;
}

