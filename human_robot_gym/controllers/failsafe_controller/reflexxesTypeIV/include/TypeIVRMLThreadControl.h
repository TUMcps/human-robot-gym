





























#ifndef z3fIm
#define z3fIm
#include <string.h>
#include <math.h>
#ifdef bzQVt
#include <pthread.h>
#endif













































class BF1yT{public:




















BF1yT(const unsigned int&dOrl9,const unsigned int&ZVhxW){unsigned int i=
(0x1a46+1864-0x218e);this->qVJUu=dOrl9;this->NumberOfDOFs=ZVhxW;this->KkDzo=
BF1yT::YmyrZ;this->G9G_5=(unsigned int)floor((float)this->NumberOfDOFs/(float)
this->qVJUu)+(0xd61+5155-0x2183);this->tYW3k=(0x6c9+4609-0x18ca);this->XonT8=
(0x393+8391-0x245a);this->bbKnN=new unsigned int[this->qVJUu];this->opUXl=new 
unsigned int*[this->qVJUu];for(i=(0x5eb+1807-0xcfa);i<this->qVJUu;i++){(this->
opUXl)[i]=new unsigned int[this->G9G_5];memset((this->opUXl)[i],
(0x144b+3991-0x23e2),(this->G9G_5*sizeof(unsigned int)));(this->bbKnN)[i]=
(0x1023+3317-0x1d18);}
#ifdef bzQVt
pthread_mutex_init(&(this->M_L7D),NULL);pthread_cond_init(&(this->NEbTs),NULL);
pthread_cond_init(&(this->VuRcE),NULL);
#endif
}












BF1yT(const BF1yT&QxJxZ){unsigned int i=(0xa02+2590-0x1420);this->bbKnN=new 
unsigned int[QxJxZ.qVJUu];this->opUXl=new unsigned int*[QxJxZ.qVJUu];for(i=
(0x1c72+1824-0x2392);i<this->qVJUu;i++){(this->opUXl)[i]=new unsigned int[QxJxZ.
G9G_5];}*this=QxJxZ;}





~BF1yT(void){unsigned int i=(0xb4+8625-0x2265);for(i=(0x1f17+759-0x220e);i<this
->qVJUu;i++){delete[](this->opUXl)[i];}delete[]this->opUXl;delete[]this->bbKnN;
this->opUXl=NULL;this->bbKnN=NULL;
#ifdef bzQVt		
pthread_mutex_destroy(&(this->M_L7D));pthread_cond_destroy(&(this->NEbTs));
pthread_cond_destroy(&(this->VuRcE));
#endif
}











BF1yT&operator=(const BF1yT&QxJxZ){unsigned int i=(0x481+5679-0x1ab0);this->
qVJUu=QxJxZ.qVJUu;this->NumberOfDOFs=QxJxZ.NumberOfDOFs;this->KkDzo=QxJxZ.KkDzo;
this->G9G_5=QxJxZ.G9G_5;this->tYW3k=QxJxZ.tYW3k;this->XonT8=QxJxZ.XonT8;memcpy(
this->bbKnN,QxJxZ.bbKnN,(this->qVJUu*sizeof(unsigned int)));memcpy(this->opUXl,
QxJxZ.opUXl,(this->qVJUu*sizeof(unsigned int*)));for(i=(0x1e0a+434-0x1fbc);i<
this->qVJUu;i++){memcpy((this->opUXl)[i],(QxJxZ.opUXl)[i],(this->G9G_5*sizeof(
unsigned int)));}
#ifdef bzQVt
this->M_L7D=QxJxZ.M_L7D;this->NEbTs=QxJxZ.NEbTs;this->VuRcE=QxJxZ.VuRcE;
#endif
return(*this);}



















enum BI2lp{
YmyrZ=(0x65b+4406-0x1791),

nWKRz=(0x413+5703-0x1a59),

dPx0Q=(0x1445+821-0x1778),

qJ6kf=(0x69a+3804-0x1573),

eaIsF=(0x330+1611-0x93c)};





























inline bool I19V0(unsigned int*KkoiW){if((this->bbKnN)[(0x176a+2094-0x1f98)]==
(0x6f2+4923-0x1a2d)){
#ifdef bzQVt		
pthread_mutex_lock(&(this->M_L7D));
#endif
(this->tYW3k)++;
#ifdef bzQVt			
pthread_mutex_unlock(&(this->M_L7D));
#endif
return(false);}else{*KkoiW=((this->opUXl)[(0xfaf+3759-0x1e5e)])[(this->bbKnN)[
(0x3b5+5118-0x17b3)]-(0x2454+513-0x2654)];((this->bbKnN)[(0x112b+327-0x1272)])--
;return(true);}}

































inline void EQKPL(const unsigned int&lpAYg,unsigned int*KkoiW,unsigned int*uhi21
){*KkoiW=(this->opUXl)[lpAYg][(this->bbKnN)[lpAYg]-(0xce6+2642-0x1737)];((this->
bbKnN)[lpAYg])--;*uhi21=this->KkDzo;}
#ifdef bzQVt



















inline void g7v8e(const unsigned int&lpAYg){if((this->bbKnN)[lpAYg]==
(0x593+7675-0x238e)){pthread_mutex_lock(&(this->M_L7D));(this->tYW3k)++;
pthread_mutex_unlock(&(this->M_L7D));pthread_cond_signal(&(this->NEbTs));}
pthread_mutex_lock(&(this->M_L7D));while(((this->bbKnN)[lpAYg]==(0x4+2627-0xa47)
)&&(this->KkDzo!=BF1yT::eaIsF)){pthread_cond_wait(&(this->VuRcE),&(this->M_L7D))
;}pthread_mutex_unlock(&(this->M_L7D));}
#endif











inline void IHXy5(void){
#ifdef bzQVt	
pthread_mutex_lock(&(this->M_L7D));while(this->tYW3k<this->XonT8){
pthread_cond_wait(&(this->NEbTs),&(this->M_L7D));}pthread_mutex_unlock(&(this->
M_L7D));
#endif
return;}









inline void s1Izq(void){
#ifdef bzQVt	
pthread_mutex_lock(&(this->M_L7D));this->KkDzo=BF1yT::eaIsF;pthread_mutex_unlock
(&(this->M_L7D));pthread_cond_broadcast(&(this->VuRcE));
#endif
return;}











inline bool DDps5(void){bool ResultValue=false;
#ifdef bzQVt	
pthread_mutex_lock(&(this->M_L7D));ResultValue=(this->KkDzo==BF1yT::eaIsF);
pthread_mutex_unlock(&(this->M_L7D));
#endif		
return(ResultValue);}
































inline void NAzWd(const bool*SelectionVector,const unsigned int&uhi21){unsigned 
int i=(0xc67+45-0xc94),Kv9sX=(0x80c+1075-0xc3f),bxMcR=(0x970+5389-0x1e7d),ki5Oj=
(0x21a0+1207-0x2657),SSEJi=(0xcc+3437-0xe39),erNok=(0xfd5+3176-0x1c3d);for(i=
(0x1a96+1139-0x1f09);i<this->NumberOfDOFs;i++){if(SelectionVector[i]){Kv9sX++;}}
bxMcR=(unsigned int)floor((float)Kv9sX/(float)this->qVJUu);ki5Oj=Kv9sX-(bxMcR*
this->qVJUu);
#ifdef bzQVt
pthread_mutex_lock(&(this->M_L7D));
#endif
this->tYW3k=(0x79+2603-0xaa4);for(i=(0x197+9115-0x2532);i<this->qVJUu;i++){if(
ki5Oj>(0xf3b+5846-0x2611)){(this->bbKnN)[i]=bxMcR+(0xf73+4980-0x22e6);ki5Oj--;}
else{(this->bbKnN)[i]=bxMcR;}}erNok=(0x1834+1445-0x1dd9);SSEJi=
(0x1221+2916-0x1d85);for(i=(0x197b+393-0x1b04);i<this->NumberOfDOFs;i++){if(
SelectionVector[i]){(this->opUXl)[erNok][SSEJi]=i;SSEJi++;if(SSEJi==(this->bbKnN
)[erNok]){SSEJi=(0xd46+6232-0x259e);erNok++;}}}this->KkDzo=uhi21;if(Kv9sX<this->
qVJUu){this->XonT8=Kv9sX;}else{this->XonT8=this->qVJUu;}
#ifdef bzQVt		
pthread_mutex_unlock(&(this->M_L7D));pthread_cond_broadcast(&(this->VuRcE));
#endif
}









inline void dO4rW(void){
#ifdef bzQVt
pthread_mutex_lock(&(this->M_L7D));while(this->tYW3k<this->qVJUu-
(0x253+5216-0x16b2)){pthread_cond_wait(&(this->NEbTs),&(this->M_L7D));}
pthread_mutex_unlock(&(this->M_L7D));
#endif
return;}









unsigned int G9G_5;








unsigned int KkDzo;









unsigned int qVJUu;








unsigned int NumberOfDOFs;









unsigned int*bbKnN;









unsigned int**opUXl;








unsigned int XonT8;











unsigned int tYW3k;
#ifdef bzQVt







pthread_mutex_t M_L7D;






pthread_cond_t NEbTs;






pthread_cond_t VuRcE;
#endif
};
#endif

