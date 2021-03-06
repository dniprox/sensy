#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#include <signal.h>
#include "web.h"


static HandleURI requestHandler = NULL;
static pthread_t webThread;
const char *authUser = NULL;
const char *authPass = NULL;

static char *Base64Decode(const char *src)
{
    const char b64char[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    static uint8_t *b64lut = NULL;

    // Make inverse LUT if it's not there yet
    if (!b64lut) {
        b64lut = (uint8_t*)calloc(256, 1);
        if (!b64lut) return NULL;
        for (unsigned int i=0; i<64; i++)
            b64lut[(unsigned int)b64char[i]] = i;
        b64lut['='] = 64; // Sentinel
    }

    if (strlen(src)%4) return NULL; // Better be divisible by 4

    char *ret, *dest;
    ret = (char *)malloc(strlen(src)/4*3+1); // 3/4 size + terminator
    if (!ret) return NULL;
    dest = ret;
    while (*src) {
        uint8_t a = b64lut[(unsigned int)*(src++)];
        uint8_t b = b64lut[(unsigned int)*(src++)];
        uint8_t c = b64lut[(unsigned int)*(src++)];
        uint8_t d = b64lut[(unsigned int)*(src++)];
        *(dest++) = (a << 2) | ((b & 0x30) >> 4);
        if (c == 64) break;
        *(dest++) = ((b & 0x0f) << 4) | ((c & 0x3c) >> 2);
        if (d == 64) break;
        *(dest++) = ((c & 0x03) << 6) | d;
    }
    *dest = 0; // Terminate the string
    return ret;
}

// Decode URL encoded strings, aborting at the first error
char *URLDecode(char *str)
{
    char *o = (char *)malloc(strlen(str) + 1);
    char *p = o;
    while (*str) {
        if (*str == '+') {
            *(p++) = ' ';
            str++;
        } else if (*str == '%') {
            unsigned int hex;
            if ((!str[1]) || (!str[2]) || (sscanf(str+1, "%2x", &hex) != 1)) {
                *p = 0;
                return o;
            }
            *(p++) = (char) hex & 0xff;
            str += 3;
        } else *(p++) = *(str++);
    }
    *p = 0;
    return o;
}

kv_t *ParseURIParams(char *uri)
{
    // Find the first argument, if it exists
    while ((*uri) && (*uri!='?')) uri++;
    if (*uri == 0) return NULL; // No parameters
    *(uri++) = 0; // Stop uri string there
    kv_t *arg = NULL;
    int args = 0;
    while (*uri) {
        char *startArg = uri;
        // Look for end & or 0
        while ((*uri) && (*uri != '&')) uri++;
        if (*uri=='&') *(uri++) = 0;
        char *name = startArg;
        while ((*startArg) && (*startArg != '=')) startArg++;
        if (*startArg == '=') *(startArg++) = 0;
        arg = (kv_t *)realloc(arg, sizeof(kv_t) * (args+1));
        if (startArg == uri) {
            // No or empty parameter
            arg[args].key = strdup(name);
            arg[args].value = strdup("");
            args++;
        } else {
            arg[args].key = strdup(name);
            arg[args].value = URLDecode(startArg);
            args++;
        }
    }
    // Add end marker
    arg = (kv_t *)realloc(arg, sizeof(kv_t) * (args+1));
    arg[args].key = NULL;
    arg[args].value = NULL;
    return arg;
}

void FreeKV(kv_t *kv)
{
    if (!kv) return;
    kv_t *s = kv;
    while (kv->key) {
        free(kv->key);
        free(kv->value);
        kv++;
    }
    free(s);
}

const char *GetKV(kv_t *kv, const char *key)
{
    while (kv && kv->key) {
        if (!strcmp(kv->key, key)) return kv->value;
        else kv++;
    }
    return NULL;
}


static void WebError(FILE *fp, const char *ret, const char *headers, const char *body)
{
    char *errorPage = (char*)malloc(strlen(body) + 1024);
    if (!errorPage) return;
    sprintf(errorPage, "<html><head><title>%s</title></head><body><h1>%s</h1><p>%s</p></body></html>", ret, ret, body);
    fprintf(fp, "HTTP/1.1 %s\r\n", ret);
    fprintf(fp, "Server: Sensy Gateway\r\n");
    fprintf(fp, "Content-length: %d\r\n", strlen(errorPage));
    fprintf(fp, "Content-type: %s\r\n", "text/html");
    fprintf(fp, "Cache-Control: no-cache, no-store, must-revalidate\r\n");
    fprintf(fp, "Pragma: no-cache\r\n");
    fprintf(fp, "Expires: 0\r\n");
    if (headers) 
        fprintf(fp, "%s", headers);
    fprintf(fp, "\r\n");
    fwrite(errorPage, 1, strlen(errorPage), fp);
    fflush(fp);
    free(errorPage);
}


static void *WebserverThread(void *ptr)
{
    int listenSock = (int)ptr;

    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
    
    while (true) {
        struct sockaddr_in addr;
        socklen_t len = sizeof(addr);
        int fd = accept(listenSock, (struct sockaddr *)&addr, &len);
        if (fd < 0) {
            // Error, just ignore
            continue;
        }
        // FD -> FILE*
        FILE *fp = fdopen(fd, "r+");
        if (!fp) {
            // Error, just ignore
            close(fd);
            continue;
        }
        // Parse request line
        char buff[8192], method[8192], uri[8192], version[8192];
        fgets(buff, 8192, fp);
        buff[8191] = 0;
        sscanf(buff, "%s %s %s", method, uri, version);
        if (strcasecmp(method, "GET")) {
            WebError(fp, "405 Method Not Allowed", "Allow: GET\r\n", "Only GET requests supported");
            fclose(fp);
            close(fd);
            continue;
        }
        // Skip all headers, we're not smart
        char *ptr;
        char auth[8192];
        auth[0] = 0;
        do {
            ptr = fgets(buff, 8192, fp);
            buff[8191] = 0;
            if (!strncmp(buff, "Authorization:", strlen("Authorization:"))) strcpy(auth, buff);
        } while( (ptr) && (!feof(fp)) && (strcmp(buff, "\r\n")) );

        if (authUser) { // Check authentication...use pound SSL proxy, this is basic realm!
            if (!auth[0]) {
                WebError(fp, "401 Unauthorized", "WWW-Authenticate: Basic realm=\"Sensy\"\r\n", "Login required.");
                fclose(fp);
                close(fd);
                continue;
            } else {
                char b64[8192];
                if (sscanf(auth, "Authorization: Basic %s", b64)!=1) {
                    WebError(fp, "401 Unauthorized", "WWW-Authenticate: Basic realm=\"Sensy\"\r\n", "Invalid login.");
                    fclose(fp);
                    close(fd);
                    continue;
                }
                char *decode = Base64Decode(b64);
                if (!decode) {
                    WebError(fp, "401 Unauthorized", "WWW-Authenticate: Basic realm=\"Sensy\"\r\n", "Invalid login.");
                    fclose(fp);
                    close(fd);
                    continue;
                }
                char *user = decode;
                while (*decode && *decode!=':') decode++;
                *decode = 0;
                char *pass = decode+1;
                if (strcmp(user, authUser) || strcmp(pass, authPass)) {
                    free(user);
                    WebError(fp, "401 Unauthorized", "WWW-Authenticate: Basic realm=\"Sensy\"\r\n", "Invalid login.");
                    fclose(fp);
                    close(fd);
                    continue;
                }
                free(user);
                // Authorized
            }
        }
        char *output = NULL;
        bool handled = false;
        kv_t *kv = ParseURIParams(uri);
        if (requestHandler)
            handled = requestHandler(uri, kv, &output);
        FreeKV(kv);
        if (!handled || !output) {
            WebError(fp, "404 Not Found", NULL, "The requested resource was not found.");
        } else {
            fprintf(fp, "HTTP/1.1 200 OK\r\n");
            fprintf(fp, "Server: Sensy Gateway\r\n");
            fprintf(fp, "Content-length: %d\r\n", strlen(output));
            fprintf(fp, "Content-type: %s\r\n", "text/html");
            fprintf(fp, "Cache-Control: no-cache, no-store, must-revalidate\r\n");
            fprintf(fp, "Pragma: no-cache\r\n");
            fprintf(fp, "Expires: 0\r\n");
            fprintf(fp, "\r\n");
            fwrite(output, 1, strlen(output), fp);
            free(output);
        }
        fflush(fp);
        fclose(fp);
        close(fd);
    }

}

bool StartWebserver(int port, HandleURI handler, const char *user, const char *pass)
{
    // Authorized user
    authUser = user;
    authPass = pass;

    // Open socket, let us reuse the port if the GW goes down
    int sock;
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
        return false;
    int optval = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (void*)&optval, sizeof(int));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        return false;
  
    if (listen(sock, 2) < 0)
        return false; // Don't clean up bind, this is only called once per execution
  
    requestHandler = handler;

    // Now we have a FD we can listen to for requests.  Make a worker thread to do so..
    if (pthread_create(&webThread, NULL, WebserverThread, (void*)sock))
        return false;

    return true;
}

void KillWebserver()
{
    pthread_cancel(webThread);
}

