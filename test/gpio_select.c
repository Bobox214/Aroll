#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/select.h>
#include <pthread.h>

#define MEAN_NB 3 // On how many ticks are we doing a meaning.

// Global make it easier for the thread communication
long int count = 0;
int bwdCount = 0;
int fdA,fdB;

void* encoderCount(void *arg) {
    char buffer[2];
	int inc;
    fd_set fds;

    while (1) {
		int fwd=0;
		int bwd=0;
		for (int i=0;i<MEAN_NB;i++) {
			// Preparer la table des evenements exceptionnels attendus
			FD_ZERO(& fds);
			// Avec uniquement le descripteur du fichier.
			FD_SET(fdA, & fds);
			// Attente passive (pas de timeout, donc infinie...
			if (select(fdA+1, NULL, NULL, & fds, NULL) < 0) {
				perror("select");
				break;
			}
			if (read(fdB, & buffer, 2) != 2) {
				perror("read");
				break;
			}
			lseek(fdB, 0, SEEK_SET);
			if (buffer[0]=='0') fwd++;
			else                bwd++;

			if (read(fdA, & buffer, 2) != 2) {
				perror("read");
				break;
			}
			lseek(fdA, 0, SEEK_SET);
		}
		if (fwd>bwd)
			count += MEAN_NB;
		else {
			count -= MEAN_NB;
			bwdCount += MEAN_NB;
		}
    }
    return NULL;
}

int main(int argc, char * argv[])
{
	pthread_t tid;
    if (argc != 3) {
        fprintf(stderr, "usage: %s \n", argv[0]);
        exit(EXIT_FAILURE);
    }
    if ((fdA = open(argv[1], O_RDONLY)) < 0) {
        perror(argv[1]);
        exit(EXIT_FAILURE);
    }
    if ((fdB = open(argv[2], O_RDONLY)) < 0) {
        perror(argv[2]);
        exit(EXIT_FAILURE);
    }

	pthread_create(&tid, NULL, &encoderCount, NULL);
    while (1) {
		sleep(1);
		fprintf(stdout,"%ld - %d\n",count,bwdCount);
    }
    return EXIT_SUCCESS;
}
