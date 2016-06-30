#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/select.h>
#include <pthread.h>

#define MEAN_NB 3 // On how many ticks are we doing a meaning.

struct count_t {
	int fdA;
	int fdB;
	long int count;
	long int bwdCount;
};

void* encoderCount(void *arg) {
    char buffer[2];
	int inc;
    fd_set fds;
	count_t* count = (count_t*) arg;

    while (1) {
		int fwd=0;
		int bwd=0;
		for (int i=0;i<MEAN_NB;i++) {
			// Preparer la table des evenements exceptionnels attendus
			FD_ZERO(& fds);
			// Avec uniquement le descripteur du fichier.
			FD_SET(count->fdA, & fds);
			// Attente passive (pas de timeout, donc infinie...
			if (select(count->fdA+1, NULL, NULL, & fds, NULL) < 0) {
				perror("select");
				break;
			}
			if (read(count->fdB, & buffer, 2) != 2) {
				perror("read");
				break;
			}
			lseek(count->fdB, 0, SEEK_SET);
			if (buffer[0]=='0') fwd++;
			else                bwd++;

			if (read(count->fdA, & buffer, 2) != 2) {
				perror("read");
				break;
			}
			lseek(count->fdA, 0, SEEK_SET);
		}
		if (fwd>bwd)
			count->count += MEAN_NB;
		else {
			count->count -= MEAN_NB;
			count->bwdCount += MEAN_NB;
		}
    }
    return NULL;
}

int main(int argc, char * argv[])
{
	pthread_t tid;
	count_t* count = new count_t();
    if (argc != 3) {
        fprintf(stderr, "usage: %s \n", argv[0]);
        exit(EXIT_FAILURE);
    }
    if ((count->fdA = open(argv[1], O_RDONLY)) < 0) {
        perror(argv[1]);
        exit(EXIT_FAILURE);
    }
    if ((count->fdB = open(argv[2], O_RDONLY)) < 0) {
        perror(argv[2]);
        exit(EXIT_FAILURE);
    }
	count->count = 0;
	count->bwdCount = 0;

	pthread_create(&tid, NULL, &encoderCount, (void*)count);
    while (1) {
		sleep(1);
		fprintf(stdout,"%ld - %ld\n",count->count,count->bwdCount);
    }
    return EXIT_SUCCESS;
}
