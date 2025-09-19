#include "ervp_printf.h"
#include "ervp_core_id.h"
#include "ervp_multicore_synch.h"
#include "ervp_variable_allocation.h"
#define ITERATIONS 10

volatile int fibo[ITERATIONS] NOTCACHED_DATA;
int fib(int i) {
    return (i > 1) ? fib(i-1) + fib(i-2) : i;
}

int main() {
	int i;

	acquire_lock(0);

	printf("fibonacci starting...\n");

	for(i = 0; i < ITERATIONS; i++){
		fibo[i] = fib(i);
		printf("fib(%03d) = %d\n", i, fibo[i]);
	}

	release_lock(0);

	return 0;
}
