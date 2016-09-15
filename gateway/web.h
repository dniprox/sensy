#ifndef _WEB_H
#define _WEB_H

typedef bool (*HandleURI)(const char *uri, char **output);
bool StartWebserver(int port, HandleURI handler);
void KillWebserver();

#endif

