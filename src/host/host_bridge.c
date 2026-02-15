#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

void *gba_host_alloc(size_t size) {
  if (size == 0) {
    return NULL;
  }
  return calloc(1, size);
}

void gba_host_free(void *ptr) { free(ptr); }

void *gba_host_memset(void *dst, int value, size_t size) {
  return memset(dst, value, size);
}

void *gba_host_memcpy(void *dst, const void *src, size_t size) {
  return memcpy(dst, src, size);
}

int gba_host_load_file(const char *path, void **out_data, size_t *out_size) {
  FILE *fp = NULL;
  long fsize = 0;
  size_t read_size = 0;
  void *buf = NULL;

  if (!path || !out_data || !out_size) {
    return -1;
  }

  fp = fopen(path, "rb");
  if (!fp) {
    return -2;
  }

  if (fseek(fp, 0, SEEK_END) != 0) {
    fclose(fp);
    return -3;
  }

  fsize = ftell(fp);
  if (fsize < 0) {
    fclose(fp);
    return -4;
  }

  if (fseek(fp, 0, SEEK_SET) != 0) {
    fclose(fp);
    return -5;
  }

  buf = malloc((size_t)fsize);
  if (!buf) {
    fclose(fp);
    return -6;
  }

  read_size = fread(buf, 1, (size_t)fsize, fp);
  fclose(fp);

  if (read_size != (size_t)fsize) {
    free(buf);
    return -7;
  }

  *out_data = buf;
  *out_size = read_size;
  return 0;
}

int gba_host_save_file(const char *path, const void *data, size_t size) {
  FILE *fp = NULL;
  size_t write_size = 0;

  if (!path || !data) {
    return -1;
  }

  fp = fopen(path, "wb");
  if (!fp) {
    return -2;
  }

  write_size = fwrite(data, 1, size, fp);
  fclose(fp);

  if (write_size != size) {
    return -3;
  }

  return 0;
}

int gba_host_file_exists(const char *path) {
  FILE *fp = NULL;

  if (!path) {
    return 0;
  }

  fp = fopen(path, "rb");
  if (!fp) {
    return 0;
  }

  fclose(fp);
  return 1;
}

uint64_t gba_host_get_ticks_us(void) {
  struct timespec ts;
  if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
    return 0;
  }
  return ((uint64_t)ts.tv_sec * 1000000ULL) + ((uint64_t)ts.tv_nsec / 1000ULL);
}

int gba_host_get_errno(void) { return errno; }
