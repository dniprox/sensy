#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#include <signal.h>


typedef bool (*HandleURI)(const char *uri, char **output);

static HandleURI requestHandler = NULL;
static pthread_t webThread;

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
        do {
            ptr = fgets(buff, 8192, fp);
        } while( (ptr) && (!feof(fp)) && (strcmp(buff, "\r\n")) );

        char *output = NULL;
        bool handled = false;
        if (requestHandler)
            handled = requestHandler(uri, &output);

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


bool StartWebserver(int port, HandleURI handler)
{
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

