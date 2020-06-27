#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <sys/types.h>
#include <sys/resource.h>
#include <sys/time.h>
// add by myself
#include <stdlib.h>
#include <limits.h>
#include <math.h>
/**
 * READ THIS DESCRIPTION
 *
 * node data structure: containing state, g, f
 * you can extend it with more information if needed
 */
typedef struct node {
	int state[16];
	int g;
	int f;
  int come;
} node_t;

/**
 * Global Variables
 */

// used to track the position of the blank in a state,
// so it doesn't have to be searched every time we check if an operator is applicable
// When we apply an operator, blank_pos is updated
int blank_pos;

// Initial node of the problem
node_t initial_node;

// Statistics about the number of generated and expendad nodes
unsigned long generated;
unsigned long expanded;


/**
 * The id of the four available actions for moving the blank (empty slot). e.x.
 * Left: moves the blank to the left, etc.
 */

#define LEFT 0
#define RIGHT 1
#define UP 2
#define DOWN 3

/*
 * Helper arrays for the applicable function
 * applicability of operators: 0 = left, 1 = right, 2 = up, 3 = down
 */
int ap_opLeft[]  = { 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1 };
int ap_opRight[]  = { 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0 };
int ap_opUp[]  = { 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
int ap_opDown[]  = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0 };
int *ap_ops[] = { ap_opLeft, ap_opRight, ap_opUp, ap_opDown };


/* print state */
void print_state( int* s )
{
	int i;

	for( i = 0; i < 16; i++ )
		printf( "%2d%c", s[i], ((i+1) % 4 == 0 ? '\n' : ' ') );
}

void printf_comma (long unsigned int n) {
    /*if (n < 0) {
        printf ("-");
        printf_comma (-n);
        return;
    }*/
    if (n < 1000) {
        printf ("%lu", n);
        return;
    }
    printf_comma (n/1000);
    printf (",%03lu", n%1000);
}

/* return the sum of manhattan distances from state to goal */
int manhattan( int* state ) {

  /* for computing the distance from this state to the end state*/

	// x, y represent the position in the 4*4 format
	// x0, y0 represent the position of each tile in the end state
	// x1, y1 represent the position of each tile in this state
	int sum = 0, i, x0, y0, x1, y1;

  for (i = 0; i < 16; i++) {
		x0 = i / 4;
    y0 = i % 4;
    x1 = state[i] / 4;
    y1 = state[i] % 4;

		// avoid addition when state[i] == 0, which is the blank tile
    if(state[i]) {

			// manhattan distance equals to the sum of the absolute value of the
			// difference between the position the tile in this state and the final state
			sum += (abs(x0 - x1) + abs(y0 - y1));
		}
	}
	return( sum );
}


/* return 1 if op is applicable in state, otherwise return 0 */
int applicable( int op, int come) {

	// avoid repeat movement of the blank tile
	if(come == LEFT && op == RIGHT) return 0;
  if(come == RIGHT && op == LEFT) return 0;
  if(come == UP && op == DOWN) return 0;
  if(come == DOWN && op == UP) return 0;

  return( ap_ops[op][blank_pos]);
}


/* apply operator */
void apply( node_t* n, int op )
{
	int t;

	//find tile that has to be moved given the op and blank_pos
	t = blank_pos + (op == 0 ? -1 : (op == 1 ? 1 : (op == 2 ? -4 : 4)));

	//apply op
	n->state[blank_pos] = n->state[t];
	n->state[t] = 0;

	//update blank pos
	blank_pos = t;
}

/* Recursive IDA */
node_t* ida( node_t* node, int threshold, int* newThreshold )
{
	int i;
  node_t* r;
	node_t* node_ = (node_t*)malloc(sizeof(node_t));

	/* Algorithm in Figure 2 of handout
	*/

	// i represent the 4 moving directions
  for(i = 0; i < 4; i ++) {
		int temp_blank_pos = blank_pos;

		// check whether the blank_pos can be swap to the direction represented by i
    if (applicable(i, node->come)) {

			// if it can be swaped, one node is generated
			generated += 1;
      memcpy(node_->state, node->state, sizeof(int) * 16);
      node_->g = node->g;
      node_->f = node->f;

      // update the state od the node
      apply(node_, i);

			// update the value of g and f of the node
      node_->g = node->g + 1;
      node_->f = node_->g + manhattan(node_->state);
      node_->come = i;

      // update the newThreshold  and back to NULL if value f of node_ is greater than threshold
			// since it shows we should not explore this path further
      if (node_->f > threshold) {
          if(node_->f < *newThreshold) {
						*newThreshold = node_->f;
					}
					blank_pos = temp_blank_pos;
					return NULL;
				}

			else {

				// back to the node_ if it is already at the final state
				if (manhattan(node_->state) == 0) {
					return node_;
				}

				expanded += 1;
			 	r = ida(node_, threshold, newThreshold);

        // r is not NULL means a suitable path is found
        if (r != NULL) {
					return r;
				}
			}
		}

		// value of blank_pos needs to be changed back since it is a global variable
		blank_pos = temp_blank_pos;
	}
	free(node_);
	node_ = NULL;
	return( NULL );
}


/* main IDA control loop */
int IDA_control_loop(  ){
	node_t* r = NULL;
	node_t* node = (node_t*)malloc(sizeof(node_t));

	int threshold, newThreshold;

	/* initialize statistics */
	generated = 0;
	expanded = 0;

	/* compute initial threshold B */
	initial_node.f = threshold = manhattan( initial_node.state );
  initial_node.g = 0;
	printf( "Initial Estimate = %d\nThreshold = ", threshold );

	/* Algorithm in Figure 1 of handout
	*/
  do {
		printf("%d ", threshold);

		// reset the newThreshold to be INT_MAX
    newThreshold = INT_MAX;
    memcpy(node->state, initial_node.state, sizeof(int) * 16);
    node->g = initial_node.g;
    node->f = initial_node.f;
    r = ida(node, threshold, &newThreshold);
    if( r == NULL ) {

			// update the threshold (the depth of searching)
			threshold = newThreshold;
		}
	} while (r == NULL);

	if(r) {
		return r->g;
	}
	else {
		return -1;
	}
}


static inline float compute_current_time()
{
	struct rusage r_usage;

	getrusage( RUSAGE_SELF, &r_usage );
	float diff_time = (float) r_usage.ru_utime.tv_sec;
	diff_time += (float) r_usage.ru_stime.tv_sec;
	diff_time += (float) r_usage.ru_utime.tv_usec / (float)1000000;
	diff_time += (float) r_usage.ru_stime.tv_usec / (float)1000000;
	return diff_time;
}

int main( int argc, char **argv )
{
	int i, solution_length;

	/* check we have a initial state as parameter */
	if( argc != 2 )
	{
		fprintf( stderr, "usage: %s \"<initial-state-file>\"\n", argv[0] );
		return( -1 );
	}


	/* read initial state */
	FILE* initFile = fopen( argv[1], "r" );
	char buffer[256];

	if( fgets(buffer, sizeof(buffer), initFile) != NULL ){
		char* tile = strtok( buffer, " " );
		for( i = 0; tile != NULL; ++i )
			{
				initial_node.state[i] = atoi( tile );
				blank_pos = (initial_node.state[i] == 0 ? i : blank_pos); // get the blank postion
				//initial_node.state[i] = i;
				//blank_pos = (initial_node.state[i] == 0 ? i : blank_pos); // get the blank postion
                tile = strtok( NULL, " " );
			}
	}
	else{
		fprintf( stderr, "Filename empty\"\n" );
		return( -2 );

	}
	if( i != 16 )
	{
		fprintf( stderr, "invalid initial state\n" );
		return( -1 );
	}

	/* initialize the initial node */
	initial_node.g=0;
	initial_node.f=0;

	print_state( initial_node.state );


	/* solve */
	float t0 = compute_current_time();

	solution_length = IDA_control_loop();

	float tf = compute_current_time();

	/* report results */
	printf( "\nSolution = %d\n", solution_length);
	printf( "Generated = ");
	printf_comma(generated);
	printf("\nExpanded = ");
	printf_comma(expanded);
	printf( "\nTime (seconds) = %.2f\nExpanded/Second = ", tf-t0 );
	printf_comma((unsigned long int) expanded/(tf+0.00000001-t0));
	printf("\n\n");

	/* aggregate all executions in a file named report.dat, for marking purposes */
	FILE* report = fopen( "report.dat", "a" );

	fprintf( report, "%s", argv[1] );
	fprintf( report, "\n\tSoulution = %d, Generated = %lu, Expanded = %lu", solution_length, generated, expanded);
	fprintf( report, ", Time = %f, Expanded/Second = %f\n\n", tf-t0, (float)expanded/(tf-t0));
	fclose(report);

	return( 0 );
}
