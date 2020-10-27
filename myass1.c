/* Solution to comp20005 Assignment 1, 2019 semester 1.

   Authorship Declaration:

   (1) I certify that the program contained in this submission is completely
   my own individual work, except where explicitly noted by comments that
   provide details otherwise.  I understand that work that has been developed
   by another student, or by me in collaboration with other students,
   or by non-students as a result of request, solicitation, or payment,
   may not be submitted for assessment in this subject.  I understand that
   submitting for assessment work developed by or in collaboration with
   other students or non-students constitutes Academic Misconduct, and
   may be penalized by mark deductions, or by other penalties determined
   via the University of Melbourne Academic Honesty Policy, as described
   at https://academicintegrity.unimelb.edu.au.

   (2) I also certify that I have not provided a copy of this work in either
   softcopy or hardcopy or any other form to any other student, and nor will
   I do so until after the marks are released. I understand that providing
   my work to other students, regardless of my intention or any undertakings
   made to me by that other student, is also Academic Misconduct.

   (3) I further understand that providing a copy of the assignment
   specification to any form of code authoring or assignment tutoring
   service, or drawing the attention of others to such services and code
   that may have been made available via such a service, may be regarded
   as Student General Misconduct (interfering with the teaching activities
   of the University and/or inciting others to commit Academic Misconduct).
   I understand that an allegation of Student General Misconduct may arise
   regardless of whether or not I personally make use of such solutions
   or sought benefit from such actions.

   Signed by: [Aditya Prawira, Student_ID:874615]
   Dated:     [April 20th, 2019]

*/

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>


/*Character constant*/
#define PERCENT '%'
#define NEWLINE '\n'

/*Numerical constant*/
#define MAXDATA 999 				/* maximum data lines */
#define MAXCHAR 10					/* maximum character in the header */
#define SQUARE 2.0					/* power of 2 */
#define DRONE_VELOCITY 4.2 			/* constant velocity used by the drone */
#define ORIGIN_POINT 0				/* origin point (0,0) */
#define PATH 2.0					/* same distance executed twice, for delivering and returning */
#define DRONE_MASS 3.8				/* mass of drone */
#define FULL_BAT 100.0				/* battery is initally full */
#define MAXMASS 5.8					/* maximum load the drone can carry */
#define SAFETY_CONSTANT 6300		/* value of safety constant used in the formula given */
#define STAGE2 2

/*Defines below used to indicate output of stage 3 and 4 as 
both stages use the same approach */
#define STAGE3 3
#define STAGE4 4


typedef char header_t[MAXCHAR+1];

/* function prototypes */
int get_char(header_t CH, int maxchar);

void delivered_packages(int A[], int package_num);
int is_delivered_packages(int A[], int package_num, int lines);
int all_delivered(int A[], int n, int lines);

void distance_from_centroid(double x, double y, 
	double X[], double Y[], double A[], int lines);

void fiiling_array(double R_LOAD[], double MASS[], double B_OUT[], 
	double B_RET[], double B_CONS[], double DIST[], 
	int DEST[], int n);

void range_load(double R_LOAD[], double MASS[], int n);
void deliver_batt(double B_OUT[], double DIST[], 
	double R_LOAD[], int n);

void batt_return(double B_RET[], double DIST[], int n);
void consumed_battery(double B_OUT[], double B_RET[], 
	double B_CONS[], int n);

void destination(int DESTINATION[], int n);
double tot_time(double DISTANCE[], int n);
double tot_distance(double DISTANCE[], int n);

void print_stage1(double X[], double Y[], double MASS[], 
	double tot_mass, int m, int n);

void sequential_delivery(double B_CONS[], double B_OUT[], 
	double B_RET[], double MASS[], double DIST[], double batt_remain,  
	double tot_dist, double tot_time, int batt_num, int stage, int n);

void non_sequential_delivery(double B_CONS[], double B_OUT[], double B_RET[], 
	double DIST[], int DEST[], double batt_remain, double tot_dist, 
	double tot_time, int batt_num, int n, int stage);

/* main program computes STAGE 1 and calling 
function protoypes to compute STAGE2 - STAGE4 */
int 
main(int argc, char *argv[]){
	header_t chars;
	int DESTINATION[MAXDATA];
	int lines = 1, i = 0, total_batteries = 1;

	double x, y, mass;
	double batt_remain;
	double centroid_x = 0, centroid_y = 0;
	double tot_cntr_time, tot_cntr_distance;
	double total_distance, total_time, total_mass;

	double RANGE_LOAD[MAXDATA];
	double CENTROID_DISTANCE[MAXDATA];
	double BAT_OUT[MAXDATA], BAT_RET[MAXDATA];
	double X[MAXDATA], Y[MAXDATA], MASS[MAXDATA];
	double DISTANCE[MAXDATA], BATT_CONS[MAXDATA];

	total_distance = total_time = total_mass = 0.0;

	/*STAGE1 output*/
	/* guard is used to takes header as input and ignoring it */
	while(get_char(chars, MAXCHAR) != EOF){
		/*Take 3 data values from the first row into an array*/
		scanf("%lf%lf%lf",&x,&y,&mass);
		X[i] = x; Y[i] = y; MASS[i] = mass;
		total_mass += mass;
		DISTANCE[i] = sqrt(pow(X[i]-ORIGIN_POINT, SQUARE) + 
			pow(Y[i]-ORIGIN_POINT, SQUARE));

		/* preparing centroid for STAGE 4 */
		centroid_x += X[i];
		centroid_y += Y[i];
		/*Take values from the remaining rows into an array*/
		while(scanf("%lf%lf%lf",&x,&y,&mass)==3 && i<MAXDATA){
			i += 1;
			/* 'lines' variable indicates number of data readed into the program
			excluding header and equivalent to number of packages to 
			be delivered by drone */
			lines += 1;
			X[i] = x; Y[i] = y; MASS[i] = mass;
			centroid_x += X[i];
			centroid_y += Y[i];
			DISTANCE[i] = sqrt(pow(X[i]-ORIGIN_POINT, SQUARE) + 
				pow(Y[i]-ORIGIN_POINT, SQUARE));
			total_mass += mass;
		}
	}
	print_stage1(X, Y, MASS, total_mass, i, lines);
	/*END OF STAGE1*/

	/*STAGE2 output*/
	batt_remain = FULL_BAT;

	/* Safe values of drone's range, battery out, battery return, 
	distance traveled, and other elements used to compute the stage */
	fiiling_array(RANGE_LOAD, MASS, 
		BAT_OUT, BAT_RET, BATT_CONS, 
		DISTANCE, DESTINATION, lines);

	total_time = tot_time(DISTANCE, lines);
	total_distance = tot_distance(DISTANCE, lines);

	sequential_delivery(BATT_CONS, BAT_OUT, BAT_RET, MASS, DISTANCE, 
		batt_remain, total_distance, total_time, total_batteries, STAGE2, lines);

	/*STAGE3 ouput*/
	batt_remain = FULL_BAT;
	total_batteries = 1;

	/* total distance and time travel for STAGE 2 
	and STAGE 3 are the same */
	non_sequential_delivery(BATT_CONS, BAT_OUT, BAT_RET, 
		DISTANCE, DESTINATION, batt_remain, total_distance, 
		total_time, total_batteries, lines, STAGE3);

	/*STAGE4 output*/
	total_batteries = 1;
	batt_remain = FULL_BAT;

	/* compute and print centroid position based on formula given */
	printf("S4, centroid location x=%6.1lfm, "
		"y=%6.1lfm\n", centroid_x/lines, centroid_y/lines);
	distance_from_centroid(centroid_x, centroid_y, X, Y, 
		CENTROID_DISTANCE, lines);

	/* refill array of battery conditions and drone's destination 
	with new values observed from centroid position */
	fiiling_array(RANGE_LOAD, MASS, BAT_OUT, 
		BAT_RET, BATT_CONS, CENTROID_DISTANCE, 
		DESTINATION, lines);

	/* compute total distance and time travel from centroid position */
	tot_cntr_time = tot_time(CENTROID_DISTANCE, lines);
	tot_cntr_distance = tot_distance(CENTROID_DISTANCE, lines);

	/* compute non-sequential delivery for STAGE4 */
	non_sequential_delivery(BATT_CONS, BAT_OUT, BAT_RET, 
		CENTROID_DISTANCE, DESTINATION, batt_remain, tot_cntr_distance, 
		tot_cntr_time , total_batteries, lines, STAGE4);
	printf("Ta daa!\n");

	/* and done */
	return 0;
}

/* function below is used to read header of the data's table */
#include <ctype.h>

int
get_char(char CH[], int maxchar){
	int ch, i = 0;
	while((ch = getchar()) != EOF && !isalpha(ch) && ch != NEWLINE){
		/*do nothing to non-alphabetical characters and NEWLINE*/
	}
	if(ch == EOF){
		return EOF;
	}
	CH[i+=1] = ch;
	while(i<maxchar && (ch = getchar()) != EOF && isalpha(ch) && ch == NEWLINE){
		CH[i+=1]=ch;
	}
	/* ending string array with no bite */
	CH[i] = '\0';
	return 0;
}

/* function is used to delete package that has been delivered */
void
delivered_packages(int A[], int package_num){
	/* REPLACED delivered package number by value above
	MAXDATA to indicate that packages is delivered */
	A[package_num] = MAXDATA + 1;	
}

/* function computes as guard for delivered pakcages */
int
is_delivered_packages(int A[], int package_num, int lines){
	int i, delivered = 0;
	for(i = 0; i<lines;i++){
		if(package_num == A[i]){
			delivered+=1;
		}
	}
	if(delivered){
		return 1;
	}else{
		return 0;
	}
}

/*Function below used to check whether all 
packages have been delivered or not*/
int
all_delivered(int A[], int n, int lines){
	int i, total_delivered = 0;
	for(i = 0; i<lines;i++){
		if(A[i] == n){
			total_delivered += 1;
		}
	}
	if(total_delivered == lines){
		return 1;
	}else{
		return 0;
	}
}

/* compute distance from centroid position */
void
distance_from_centroid(double x, double y, 
	double X[], double Y[], double A[], int lines){

	int i = 0;
	for(i = 0; i < lines; i++){
		A[i] = sqrt(pow(X[i]-(x/lines), SQUARE) + 
			pow(Y[i]-(y/lines), SQUARE));
	}
}

/* compute drone's delivery range */
void
range_load(double R_LOAD[], double MASS[], int n){
	int i;
	for(i = 0; i<n; i++){
		R_LOAD[i]= SAFETY_CONSTANT/(DRONE_MASS + MASS[i]);
	}
}

/*  battery consumed for out delivery */
void
deliver_batt(double B_OUT[], double DIST[], double R_LOAD[], int n){
	int i;
	/*compute percentage in array*/
	for(i =0; i<n; i++){
		B_OUT[i] = 100*(DIST[i]/R_LOAD[i]);
	}
}

/* compute battery consumed for returning drone */
void
batt_return(double B_RET[], double DIST[], int n){
	int i;
	double range_no_load;
	range_no_load = SAFETY_CONSTANT/DRONE_MASS;
	/*compute percentage in array*/
	for(i = 0; i<n; i++){
		B_RET[i] = 100*(DIST[i]/range_no_load);
	}
}

/* compute total consumed battery*/
void
consumed_battery(double B_OUT[], double B_RET[], double B_CONS[], int n){
	int i;
	for(i = 0; i<n;i++){
		B_CONS[i] = B_OUT[i] + B_RET[i];
	}
}

/* compute destination */
void
destination(int DEST[], int n){
	int i;
	for(i = 0; i<n; i++){
		DEST[i] = i;
	}
}

/* compute total time of delivery */
double
tot_time(double DIST[], int n){
	int i;
	double total_time = 0, time;
	for(i = 0; i<n;i++){
		time = PATH*(DIST[i]/DRONE_VELOCITY);
		total_time += time;
	}
	return total_time;
}

/* compute total distance of delivery */
double
tot_distance(double DIST[], int n){
	int i;
	double total_distance = 0;
	for(i=0;i<n;i++){
		total_distance += PATH*DIST[i];
	}
	return total_distance;
}

/* Store values into an array of the corresponding variable */
void
fiiling_array(double R_LOAD[], double MASS[], double B_OUT[], 
	double B_RET[], double B_CONS[], double DIST[], 
	int DEST[], int n){

	range_load(R_LOAD, MASS, n);
	deliver_batt(B_OUT, DIST, R_LOAD, n);
	batt_return(B_RET, DIST, n);
	consumed_battery(B_OUT, B_RET, B_CONS, n);
	destination(DEST, n);

}

/*printf the first stage*/
void 
print_stage1(double X[], double Y[], double MASS[], 
	double tot_mass, int m, int n){
	printf("S1, total data lines:%4d\n", n);
	printf("S1, first data line :	x=%6.1lf, y=%6.1lf, "
		"kg=%4.2f\n", X[0], Y[0], MASS[0]);
	printf("S1, final data line :	x=%6.1lf, y=%6.1lf, "
		"kg=%4.2lf\n", X[m], Y[m], MASS[m]);		
	printf("S1, total to deliver: %.2lf kg\n", tot_mass);
	printf("\n");
}

/* compute ands print out of STAGE 2 based on sequential delivery */
void
sequential_delivery(double B_CONS[], double B_OUT[], 
	double B_RET[], double MASS[], double DIST[], double batt_remain, 
	double tot_dist, double tot_time, int batt_num, int stage, int n){

	int i;
	/*Starting array from i = 1 to compare before and after value*/
	for(i = 1; i<=n; i++){
		printf("S%d, package=%4d", stage, i-1);
		batt_remain -= B_CONS[i-1];

		/*Print Error if package's mass exceed Drone's 
		capacity or if battery consumed is > 100%*/
		if(MASS[i-1]>MAXMASS || B_CONS[i-1] > FULL_BAT){
			printf("ERROR! DATA EXCEED DRONE'S MAXIMUM CAPACITY!\n");
			exit(EXIT_FAILURE);
		}
		printf(", distance= %5.1lfm, battery out=%4.1lf%c, "
			"battery ret=%4.1lf%c\n", DIST[i-1], B_OUT[i-1], 
			PERCENT, B_RET[i-1], PERCENT);
		/*Analyze all array values before the last value*/
		if(i < n){
			/* checking whether the current battery can be 
			use for the next delivery or not */
			if(batt_remain < B_CONS[i]){
				printf("S%d, change the battery\n", stage);
				batt_num += 1;
				batt_remain = FULL_BAT;
			}else{
				/* If not, use the last remaining battery 
				capacity for the next delivery */
				batt_remain -= B_CONS[i];
			}
		}
	}
	/*printf total battery, distance and times for overall delivery*/
	printf("S%d, total batteries required:%4d\n", stage, batt_num);
	printf("S%d, total flight distance=%4.1lf meters, "
		"total flight time=%4.0lf seconds\n", stage, tot_dist, tot_time);
	printf("\n");
	/*END OF STAGE2*/
}

/* compute and print outputs of non sequntial delivery */
void
non_sequential_delivery(double B_CONS[], double B_OUT[], double B_RET[], 
	double DIST[], int DEST[], double batt_remain, double tot_dist, 
	double tot_time, int batt_num, int n, int stage){
	int i, cycle = 0;
	i = 0;
	while(i<n){
		if(is_delivered_packages(DEST, i, n)){
			if(batt_remain > B_CONS[i]){
				printf("S%d, package=%4d", stage, i);
				printf(", distance= %5.1lfm, battery out=%4.1lf%c, "
					"battery ret=%4.1lf%c\n", DIST[i], B_OUT[i], 
					PERCENT, B_RET[i], PERCENT);
				delivered_packages(DEST, i);
				batt_remain -= B_CONS[i];
			}
		}
		i++;
		/*Guards below used to clarify if all packages has been delivered or not 
		while the program searching for battery friendly package destination*/
		cycle += 1;
		if(cycle == n && all_delivered(DEST, (MAXDATA +1), n)){
			break;
		}else if(cycle == n && !all_delivered(DEST, (MAXDATA +1), n)){
			i = 0;
			cycle = 0;
			batt_remain = FULL_BAT;
			batt_num += 1;
			printf("S%d, change the battery\n", stage);
		}
	}
	printf("S%d, total batteries required:%4d\n", stage, batt_num);
	printf("S%d, total flight distance=%4.1lf meters, "
		"total flight time=%4.0lf seconds\n", stage, tot_dist, tot_time);
	printf("\n");
}
/* programming if fun */

