#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* Command Queue */
typedef struct Task {
	uint8_t command_code; // Photo, Process, Get Data, etc
	uint8_t data_len[4]; // Command-specific parameter
	uint8_t *data;
	struct Task *next;
} task;

task *createTask(uint8_t command_code, uint8_t *data_len, uint8_t *data) {
	/* Generate new task */
	task *newTask = (task*)malloc(sizeof(task)); // Command type and parameter
	newTask->command_code = command_code;
	memcpy(newTask->data_len, data_len, sizeof(*data_len));

	int dataLength = sizeof(data); // Data
	newTask->data = (uint8_t*)malloc(dataLength);
	memcpy(newTask->data, data, dataLength);

	newTask->next = NULL;

	return newTask;
}

typedef struct Queue {
	task *front;
	task *back;
	uint8_t numTasks;
} queue;

void initQueue(queue *que) {
	que->numTasks = 0;
	que->front = NULL;
	que->back = NULL;
}

task *peekQueue(queue *que) {
	return que->front;
}

int emptyQueue(queue *que) {
	if (que->numTasks == 0)
		return 1;
	else
		return 0;
}

void enqueue(task *newTask, queue *que) {
	if (emptyQueue(que)) {
		que->front = newTask;
		que->back = newTask;
	}
	else {
		que->back = newTask;
	}

	que->numTasks++;
}

void dequeue(queue *que) {
	task *temp = que->front;
	que->front = que->front->next;
	free(temp);
	que->numTasks--;
}

/* USER CODE END 0 */

int main(void)
{
	/* Sample Task Generation */
	uint8_t *dataA = (uint8_t*)calloc(150, 1);
	long time = 123456789;
	uint8_t idA[2] = { 12,8 };
	memcpy(dataA, &time, sizeof(time));
	memcpy(dataA + sizeof(time), idA, sizeof(idA));
	uint8_t lenA[4] = { 1,1,1,1 };
	task *taskA = createTask(1, lenA, dataA);

	/*
	uint8_t *dataB = (uint8_t*)calloc(6, 1);
	uint8_t idB[2] = { 12,8 };
	uint8_t lenB[4] = { 1,1,1,1 };
	task *taskB = createTask(2, 6, dataB);

	uint8_t *dataC = (uint8_t*)calloc(2, 1);
	task *taskC = createTask(3, 2, dataC);
	*/

	/* Allocate and Initialize Queue */
	queue *jobs = (queue*)malloc(sizeof(queue));
	initQueue(jobs);

	queue *errJobs = (queue*)malloc(sizeof(queue));
	initQueue(errJobs);

	/* Queue Sample Tasks */
	enqueue(taskA, jobs);

	/* Define Error-Codes */
	uint8_t errCodes[12], i;
	for (i = 0; i<12; i++) {
		errCodes[i] = 129 + i;
	}

	uint8_t succCodes[7];
	for (i = 0; i<7; i++) {
		succCodes[i] = 65 + i;
	}

	/* Wait for a response -----------*/
	task *currJob = peekQueue(jobs);
	uint8_t reply = 66; // 1-byte response code (Success, Error)
	uint8_t success_code;

	/* Determine success code for given command-type */
	success_code = succCodes[currJob->command_code - 1];

	if (reply == success_code) {
		// Receive n-bytes depending on job-type completed
		printf("Success!\n");
	}
	else { // Remove job from queue, add it to error queue
		enqueue(currJob, errJobs);
		dequeue(jobs);
		printf("Error\n");
	}

}