#ifndef _WEB_H
#define _WEB_H

typedef struct {
    char *key;
    char *value;
} kv_t;

typedef bool (*HandleURI)(const char *uri, kv_t *params, char **output);
const char *GetKV(kv_t *kv, const char *key);
bool StartWebserver(int port, HandleURI handler, const char *authUser, const char *authPass);
void KillWebserver();

#endif

