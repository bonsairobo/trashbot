#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <arpa/inet.h>
#include <time.h>
#include <string.h>
#include <sys/stat.h>

#include "ioutils.h"

char *
get_unique_filename (const char *path, const char *basename,
                     uint8_t time_prefix, const char *extension)
{
    char prefix[256];

    if (time_prefix) {
        time_t t = time (NULL);
        struct tm ti;
        localtime_r (&t, &ti);

        if (path)
            snprintf (prefix, sizeof prefix, "%s/%d-%02d-%02d-%s",
                      path, ti.tm_year+1900, ti.tm_mon+1, ti.tm_mday, basename);
        else
            snprintf (prefix, sizeof prefix, "%d-%02d-%02d-%s",
                      ti.tm_year+1900, ti.tm_mon+1, ti.tm_mday, basename);
    }
    else {
        if (path)
            snprintf (prefix, sizeof prefix, "%s/%s",
                      path, basename);
        else
            snprintf (prefix, sizeof prefix, "%s", basename);
    }

    int maxlen = 4 + sizeof prefix;
    if (extension)
        maxlen += strlen (extension);
    char *filename = malloc (maxlen);

    /* Loop through possible file names until we find one that doesn't already
     * exist.  This way, we never overwrite an existing file. */
    int res;
    int filenum = 0;
    do {
        struct stat statbuf;
        if (extension)
            snprintf (filename, maxlen, "%s.%02d.%s", prefix, filenum,
                      extension);
        else
            snprintf (filename, maxlen, "%s.%02d", prefix, filenum);
        res = stat (filename, &statbuf);
        filenum++;
    } while (res == 0 && filenum < 100);

    if (filenum >= 100) {
        free (filename);
        return NULL;
    }

    if (errno != ENOENT) {
        free (filename);
        return NULL;
    }

    return filename;
}

int
write_fully (int fd, const void *b, int len)
{
    int cnt = 0;
    int thiscnt;
    const unsigned char *bb = b;

    while (cnt<len)
    {
        thiscnt = write (fd, &bb[cnt], len-cnt);
        if (thiscnt < 0) {
            perror ("write");
            return -1;
        }
        cnt += thiscnt;
    }

    return cnt;
}

int
read_fully (int fd, void *b, int len)
{
    int cnt = 0;
    int thiscnt;
    unsigned char *bb = b;

    while (cnt < len)
    {
        thiscnt = read (fd, &bb[cnt], len-cnt);
        if (thiscnt < 0) {
            perror ("read_fully");
            return -1;
        }
        if (thiscnt == 0) {
            fprintf (stderr, "read_fully end of file\n");
            return -1;
        }

        cnt += thiscnt;
    }

    return cnt;
}

/* returns -1 on error, 0 on timeout, nchar on success. */
int
read_timeout (int fd, void *buf, int maxlen, int msTimeout)
{
    struct pollfd pfd;
    int len;
    int res;

    pfd.fd = fd;
    pfd.events = POLLIN;

    res = poll (&pfd, 1, msTimeout);
    if (res < 0) { // error
        perror ("poll");
        return -1;
    }
    if (res == 0) { // timeout
        errno = ETIMEDOUT;
        return 0;
    }

    len = read (fd, buf, maxlen);
    if (len < 0) {
        perror ("read");
        return -1;
    }
    if (len == 0) {
        fprintf (stderr, "end of file\n");
        return -1;
    }
    return len;
}

/* returns -1 on error, nchar on timeout or success. */
int
read_fully_timeout (int fd, void *bufin, int len, int msTimeout)
{
    char *buf = bufin;
    int readsofar = 0;
    int thisread = 0;

    while (readsofar<len) {
        thisread = read_timeout (fd, &buf[readsofar], len-readsofar, msTimeout);
        if (thisread == 0)
            return readsofar;
        if (thisread < 0)
            return thisread;

        readsofar += thisread;
    }

    return len;
}

// reads number of bytes available, <0 on error.
int
read_available (int fd)
{
    long avail = 0;

    if (0==ioctl (fd, FIONREAD, &avail))
        return avail;
    else
        return -1;
}

// read (and discard) all available data.
void
read_flush (int fd)
{
    int avail;
    char buf[1024];

    do {
        avail = read_available (fd);
        if (avail > 1024)
            avail = 1024;

        int res = read (fd, buf, avail);
        res++; // trick to ignore compiler warnings about unread result

    } while (avail>0);
}

// returns length of string, -1 on error, 0 on timeout.
int
read_line_timeout (int fd, void *buf_in, int maxlen, int msTimeout)
{
    int timed_out;
    return read_line_timeout_ex (fd, buf_in, maxlen, msTimeout, &timed_out);
}

int
read_line_timeout_ex (int fd, void *buf_in, int maxlen, int msTimeout, int *timed_out)
{
    int len = 0;
    int thislen;
    char *buf = buf_in;

    *timed_out = 0;

    while (len<maxlen) {
        thislen = read_timeout (fd, &buf[len], 1, msTimeout);
        if (thislen < 0)
            return thislen;
        if (thislen==0) {
            *timed_out = 1;
            return 0;
        }

        if (buf[len]=='\n' || buf[len]=='\r') {
            buf[len]=0;
            return len;
        }

        len++;
    }

    buf[len]=0;
    return len;
}

int
fwrite64 (FILE *f, int64_t v64)
{
    int32_t v = v64>>32;
    if (0 != fwrite32 (f, v))
        return -1;
    v = v64 & 0xffffffff;
    return fwrite32 (f, v);
}

int
fwrite32 (FILE *f, int32_t v)
{
    v = htonl (v);
    if (1==fwrite (&v, 4, 1, f))
        return 0;
    else
        return -1;
}

int
fread32 (FILE *f, int32_t *v32)
{
    int32_t v;

    if (1!=fread (&v, 4, 1, f))
        return -1;

    *v32 = ntohl (v);

    return 0;
}

int
fread64 (FILE *f, int64_t *v64)
{
    int32_t v1, v2;

    if (fread32 (f, &v1))
        return -1;

    if (fread32 (f, &v2))
        return -1;

    *v64 = (((int64_t) v1)<<32) | (((int64_t) v2)&0xffffffff);

    return 0;
}

