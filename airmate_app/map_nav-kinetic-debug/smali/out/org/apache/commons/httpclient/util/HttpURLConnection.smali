.class public Lorg/apache/commons/httpclient/util/HttpURLConnection;
.super Ljava/net/HttpURLConnection;
.source "HttpURLConnection.java"


# static fields
.field private static final LOG:Lorg/apache/commons/logging/Log;

.field static synthetic class$org$apache$commons$httpclient$util$HttpURLConnection:Ljava/lang/Class;


# instance fields
.field private method:Lorg/apache/commons/httpclient/HttpMethod;

.field private url:Ljava/net/URL;


# direct methods
.method static constructor <clinit>()V
    .registers 1

    .line 72
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->class$org$apache$commons$httpclient$util$HttpURLConnection:Ljava/lang/Class;

    if-nez v0, :cond_d

    const-string v0, "org.apache.commons.httpclient.util.HttpURLConnection"

    invoke-static {v0}, Lorg/apache/commons/httpclient/util/HttpURLConnection;->class$(Ljava/lang/String;)Ljava/lang/Class;

    move-result-object v0

    sput-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->class$org$apache$commons$httpclient$util$HttpURLConnection:Ljava/lang/Class;

    goto :goto_f

    :cond_d
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->class$org$apache$commons$httpclient$util$HttpURLConnection:Ljava/lang/Class;

    :goto_f
    invoke-static {v0}, Lorg/apache/commons/logging/LogFactory;->getLog(Ljava/lang/Class;)Lorg/apache/commons/logging/Log;

    move-result-object v0

    sput-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    return-void
.end method

.method protected constructor <init>(Ljava/net/URL;)V
    .registers 4
    .param p1, "url"    # Ljava/net/URL;

    .line 111
    invoke-direct {p0, p1}, Ljava/net/HttpURLConnection;-><init>(Ljava/net/URL;)V

    .line 112
    new-instance v0, Ljava/lang/RuntimeException;

    const-string v1, "An HTTP URL connection can only be constructed from a HttpMethod class"

    invoke-direct {v0, v1}, Ljava/lang/RuntimeException;-><init>(Ljava/lang/String;)V

    throw v0
.end method

.method public constructor <init>(Lorg/apache/commons/httpclient/HttpMethod;Ljava/net/URL;)V
    .registers 3
    .param p1, "method"    # Lorg/apache/commons/httpclient/HttpMethod;
    .param p2, "url"    # Ljava/net/URL;

    .line 100
    invoke-direct {p0, p2}, Ljava/net/HttpURLConnection;-><init>(Ljava/net/URL;)V

    .line 101
    iput-object p1, p0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->method:Lorg/apache/commons/httpclient/HttpMethod;

    .line 102
    iput-object p2, p0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->url:Ljava/net/URL;

    .line 103
    return-void
.end method

.method static synthetic class$(Ljava/lang/String;)Ljava/lang/Class;
    .registers 4
    .param p0, "x0"    # Ljava/lang/String;

    .line 72
    :try_start_0
    invoke-static {p0}, Ljava/lang/Class;->forName(Ljava/lang/String;)Ljava/lang/Class;

    move-result-object v0
    :try_end_4
    .catch Ljava/lang/ClassNotFoundException; {:try_start_0 .. :try_end_4} :catch_5

    return-object v0

    :catch_5
    move-exception v0

    .local v0, "x1":Ljava/lang/ClassNotFoundException;
    new-instance v1, Ljava/lang/NoClassDefFoundError;

    invoke-virtual {v0}, Ljava/lang/ClassNotFoundException;->getMessage()Ljava/lang/String;

    move-result-object v2

    invoke-direct {v1, v2}, Ljava/lang/NoClassDefFoundError;-><init>(Ljava/lang/String;)V

    throw v1
.end method


# virtual methods
.method public connect()V
    .registers 3
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 156
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.connect()"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 157
    new-instance v0, Ljava/lang/RuntimeException;

    const-string v1, "This class can only be used with alreadyretrieved data"

    invoke-direct {v0, v1}, Ljava/lang/RuntimeException;-><init>(Ljava/lang/String;)V

    throw v0
.end method

.method public disconnect()V
    .registers 3

    .line 146
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.disconnect()"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 147
    new-instance v0, Ljava/lang/RuntimeException;

    const-string v1, "Not implemented yet"

    invoke-direct {v0, v1}, Ljava/lang/RuntimeException;-><init>(Ljava/lang/String;)V

    throw v0
.end method

.method public getAllowUserInteraction()Z
    .registers 3

    .line 423
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.getAllowUserInteraction()"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 424
    new-instance v0, Ljava/lang/RuntimeException;

    const-string v1, "Not implemented yet"

    invoke-direct {v0, v1}, Ljava/lang/RuntimeException;-><init>(Ljava/lang/String;)V

    throw v0
.end method

.method public getContent()Ljava/lang/Object;
    .registers 3
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 349
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.getContent()"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 350
    new-instance v0, Ljava/lang/RuntimeException;

    const-string v1, "Not implemented yet"

    invoke-direct {v0, v1}, Ljava/lang/RuntimeException;-><init>(Ljava/lang/String;)V

    throw v0
.end method

.method public getContent([Ljava/lang/Class;)Ljava/lang/Object;
    .registers 4
    .param p1, "classes"    # [Ljava/lang/Class;
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 357
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.getContent(Class[])"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 358
    new-instance v0, Ljava/lang/RuntimeException;

    const-string v1, "Not implemented yet"

    invoke-direct {v0, v1}, Ljava/lang/RuntimeException;-><init>(Ljava/lang/String;)V

    throw v0
.end method

.method public getDefaultUseCaches()Z
    .registers 3

    .line 470
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.getDefaultUseCaches()"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 471
    new-instance v0, Ljava/lang/RuntimeException;

    const-string v1, "Not implemented yet"

    invoke-direct {v0, v1}, Ljava/lang/RuntimeException;-><init>(Ljava/lang/String;)V

    throw v0
.end method

.method public getDoInput()Z
    .registers 3

    .line 385
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.getDoInput()"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 386
    new-instance v0, Ljava/lang/RuntimeException;

    const-string v1, "Not implemented yet"

    invoke-direct {v0, v1}, Ljava/lang/RuntimeException;-><init>(Ljava/lang/String;)V

    throw v0
.end method

.method public getDoOutput()Z
    .registers 3

    .line 404
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.getDoOutput()"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 405
    new-instance v0, Ljava/lang/RuntimeException;

    const-string v1, "Not implemented yet"

    invoke-direct {v0, v1}, Ljava/lang/RuntimeException;-><init>(Ljava/lang/String;)V

    throw v0
.end method

.method public getErrorStream()Ljava/io/InputStream;
    .registers 3

    .line 137
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.getErrorStream()"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 138
    new-instance v0, Ljava/lang/RuntimeException;

    const-string v1, "Not implemented yet"

    invoke-direct {v0, v1}, Ljava/lang/RuntimeException;-><init>(Ljava/lang/String;)V

    throw v0
.end method

.method public getHeaderField(I)Ljava/lang/String;
    .registers 4
    .param p1, "position"    # I

    .line 264
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.getHeaderField(int)"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 270
    if-nez p1, :cond_14

    .line 271
    iget-object v0, p0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->method:Lorg/apache/commons/httpclient/HttpMethod;

    invoke-interface {v0}, Lorg/apache/commons/httpclient/HttpMethod;->getStatusLine()Lorg/apache/commons/httpclient/StatusLine;

    move-result-object v0

    invoke-virtual {v0}, Lorg/apache/commons/httpclient/StatusLine;->toString()Ljava/lang/String;

    move-result-object v0

    return-object v0

    .line 277
    :cond_14
    iget-object v0, p0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->method:Lorg/apache/commons/httpclient/HttpMethod;

    invoke-interface {v0}, Lorg/apache/commons/httpclient/HttpMethod;->getResponseHeaders()[Lorg/apache/commons/httpclient/Header;

    move-result-object v0

    .line 278
    .local v0, "headers":[Lorg/apache/commons/httpclient/Header;
    if-ltz p1, :cond_29

    array-length v1, v0

    if-le p1, v1, :cond_20

    goto :goto_29

    .line 282
    :cond_20
    add-int/lit8 v1, p1, -0x1

    aget-object v1, v0, v1

    invoke-virtual {v1}, Lorg/apache/commons/httpclient/Header;->getValue()Ljava/lang/String;

    move-result-object v1

    return-object v1

    .line 279
    :cond_29
    :goto_29
    const/4 v1, 0x0

    return-object v1
.end method

.method public getHeaderField(Ljava/lang/String;)Ljava/lang/String;
    .registers 5
    .param p1, "name"    # Ljava/lang/String;

    .line 214
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.getHeaderField(String)"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 217
    iget-object v0, p0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->method:Lorg/apache/commons/httpclient/HttpMethod;

    invoke-interface {v0}, Lorg/apache/commons/httpclient/HttpMethod;->getResponseHeaders()[Lorg/apache/commons/httpclient/Header;

    move-result-object v0

    .line 218
    .local v0, "headers":[Lorg/apache/commons/httpclient/Header;
    array-length v1, v0

    add-int/lit8 v1, v1, -0x1

    .local v1, "i":I
    :goto_10
    if-ltz v1, :cond_28

    .line 219
    aget-object v2, v0, v1

    invoke-virtual {v2}, Lorg/apache/commons/httpclient/Header;->getName()Ljava/lang/String;

    move-result-object v2

    invoke-virtual {v2, p1}, Ljava/lang/String;->equalsIgnoreCase(Ljava/lang/String;)Z

    move-result v2

    if-eqz v2, :cond_25

    .line 220
    aget-object v2, v0, v1

    invoke-virtual {v2}, Lorg/apache/commons/httpclient/Header;->getValue()Ljava/lang/String;

    move-result-object v2

    return-object v2

    .line 218
    :cond_25
    add-int/lit8 v1, v1, -0x1

    goto :goto_10

    .line 224
    .end local v1    # "i":I
    :cond_28
    const/4 v1, 0x0

    return-object v1
.end method

.method public getHeaderFieldKey(I)Ljava/lang/String;
    .registers 5
    .param p1, "keyPosition"    # I

    .line 235
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.getHeaderFieldKey(int)"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 241
    const/4 v0, 0x0

    if-nez p1, :cond_b

    .line 242
    return-object v0

    .line 248
    :cond_b
    iget-object v1, p0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->method:Lorg/apache/commons/httpclient/HttpMethod;

    invoke-interface {v1}, Lorg/apache/commons/httpclient/HttpMethod;->getResponseHeaders()[Lorg/apache/commons/httpclient/Header;

    move-result-object v1

    .line 249
    .local v1, "headers":[Lorg/apache/commons/httpclient/Header;
    if-ltz p1, :cond_20

    array-length v2, v1

    if-le p1, v2, :cond_17

    goto :goto_20

    .line 253
    :cond_17
    add-int/lit8 v0, p1, -0x1

    aget-object v0, v1, v0

    invoke-virtual {v0}, Lorg/apache/commons/httpclient/Header;->getName()Ljava/lang/String;

    move-result-object v0

    return-object v0

    .line 250
    :cond_20
    :goto_20
    return-object v0
.end method

.method public getIfModifiedSince()J
    .registers 3

    .line 461
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.getIfmodifiedSince()"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 462
    new-instance v0, Ljava/lang/RuntimeException;

    const-string v1, "Not implemented yet"

    invoke-direct {v0, v1}, Ljava/lang/RuntimeException;-><init>(Ljava/lang/String;)V

    throw v0
.end method

.method public getInputStream()Ljava/io/InputStream;
    .registers 3
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 127
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.getInputStream()"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 128
    iget-object v0, p0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->method:Lorg/apache/commons/httpclient/HttpMethod;

    invoke-interface {v0}, Lorg/apache/commons/httpclient/HttpMethod;->getResponseBodyAsStream()Ljava/io/InputStream;

    move-result-object v0

    return-object v0
.end method

.method public getInstanceFollowRedirects()Z
    .registers 3

    .line 321
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.getInstanceFollowRedirects()"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 322
    new-instance v0, Ljava/lang/RuntimeException;

    const-string v1, "Not implemented yet"

    invoke-direct {v0, v1}, Ljava/lang/RuntimeException;-><init>(Ljava/lang/String;)V

    throw v0
.end method

.method public getOutputStream()Ljava/io/OutputStream;
    .registers 3
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 365
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.getOutputStream()"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 366
    new-instance v0, Ljava/lang/RuntimeException;

    const-string v1, "This class can only be used with alreadyretrieved data"

    invoke-direct {v0, v1}, Ljava/lang/RuntimeException;-><init>(Ljava/lang/String;)V

    throw v0
.end method

.method public getPermission()Ljava/security/Permission;
    .registers 3
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 340
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.getPermission()"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 341
    new-instance v0, Ljava/lang/RuntimeException;

    const-string v1, "Not implemented yet"

    invoke-direct {v0, v1}, Ljava/lang/RuntimeException;-><init>(Ljava/lang/String;)V

    throw v0
.end method

.method public getRequestMethod()Ljava/lang/String;
    .registers 3

    .line 178
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.getRequestMethod()"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 179
    iget-object v0, p0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->method:Lorg/apache/commons/httpclient/HttpMethod;

    invoke-interface {v0}, Lorg/apache/commons/httpclient/HttpMethod;->getName()Ljava/lang/String;

    move-result-object v0

    return-object v0
.end method

.method public getRequestProperty(Ljava/lang/String;)Ljava/lang/String;
    .registers 4
    .param p1, "key"    # Ljava/lang/String;

    .line 499
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.getRequestProperty()"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 500
    new-instance v0, Ljava/lang/RuntimeException;

    const-string v1, "Not implemented yet"

    invoke-direct {v0, v1}, Ljava/lang/RuntimeException;-><init>(Ljava/lang/String;)V

    throw v0
.end method

.method public getResponseCode()I
    .registers 3
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 190
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.getResponseCode()"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 191
    iget-object v0, p0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->method:Lorg/apache/commons/httpclient/HttpMethod;

    invoke-interface {v0}, Lorg/apache/commons/httpclient/HttpMethod;->getStatusCode()I

    move-result v0

    return v0
.end method

.method public getResponseMessage()Ljava/lang/String;
    .registers 3
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 202
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.getResponseMessage()"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 203
    iget-object v0, p0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->method:Lorg/apache/commons/httpclient/HttpMethod;

    invoke-interface {v0}, Lorg/apache/commons/httpclient/HttpMethod;->getStatusText()Ljava/lang/String;

    move-result-object v0

    return-object v0
.end method

.method public getURL()Ljava/net/URL;
    .registers 3

    .line 291
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.getURL()"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 292
    iget-object v0, p0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->url:Ljava/net/URL;

    return-object v0
.end method

.method public getUseCaches()Z
    .registers 3

    .line 442
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.getUseCaches()"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 443
    new-instance v0, Ljava/lang/RuntimeException;

    const-string v1, "Not implemented yet"

    invoke-direct {v0, v1}, Ljava/lang/RuntimeException;-><init>(Ljava/lang/String;)V

    throw v0
.end method

.method public setAllowUserInteraction(Z)V
    .registers 4
    .param p1, "isAllowInteraction"    # Z

    .line 413
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.setAllowUserInteraction(boolean)"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 414
    new-instance v0, Ljava/lang/RuntimeException;

    const-string v1, "This class can only be used with alreadyretrieved data"

    invoke-direct {v0, v1}, Ljava/lang/RuntimeException;-><init>(Ljava/lang/String;)V

    throw v0
.end method

.method public setDefaultUseCaches(Z)V
    .registers 4
    .param p1, "isUsingCaches"    # Z

    .line 479
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.setDefaultUseCaches(boolean)"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 480
    new-instance v0, Ljava/lang/RuntimeException;

    const-string v1, "This class can only be used with alreadyretrieved data"

    invoke-direct {v0, v1}, Ljava/lang/RuntimeException;-><init>(Ljava/lang/String;)V

    throw v0
.end method

.method public setDoInput(Z)V
    .registers 4
    .param p1, "isInput"    # Z

    .line 375
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.setDoInput()"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 376
    new-instance v0, Ljava/lang/RuntimeException;

    const-string v1, "This class can only be used with alreadyretrieved data"

    invoke-direct {v0, v1}, Ljava/lang/RuntimeException;-><init>(Ljava/lang/String;)V

    throw v0
.end method

.method public setDoOutput(Z)V
    .registers 4
    .param p1, "isOutput"    # Z

    .line 394
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.setDoOutput()"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 395
    new-instance v0, Ljava/lang/RuntimeException;

    const-string v1, "This class can only be used with alreadyretrieved data"

    invoke-direct {v0, v1}, Ljava/lang/RuntimeException;-><init>(Ljava/lang/String;)V

    throw v0
.end method

.method public setIfModifiedSince(J)V
    .registers 5
    .param p1, "modificationDate"    # J

    .line 451
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.setIfModifiedSince(long)"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 452
    new-instance v0, Ljava/lang/RuntimeException;

    const-string v1, "This class can only be used with alreadyretrieved data"

    invoke-direct {v0, v1}, Ljava/lang/RuntimeException;-><init>(Ljava/lang/String;)V

    throw v0
.end method

.method public setInstanceFollowRedirects(Z)V
    .registers 4
    .param p1, "isFollowingRedirects"    # Z

    .line 312
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.setInstanceFollowRedirects(boolean)"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 313
    new-instance v0, Ljava/lang/RuntimeException;

    const-string v1, "This class can only be used with alreadyretrieved data"

    invoke-direct {v0, v1}, Ljava/lang/RuntimeException;-><init>(Ljava/lang/String;)V

    throw v0
.end method

.method public setRequestMethod(Ljava/lang/String;)V
    .registers 4
    .param p1, "method"    # Ljava/lang/String;
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/net/ProtocolException;
        }
    .end annotation

    .line 330
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.setRequestMethod(String)"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 331
    new-instance v0, Ljava/lang/RuntimeException;

    const-string v1, "This class can only be used with alreadyretrieved data"

    invoke-direct {v0, v1}, Ljava/lang/RuntimeException;-><init>(Ljava/lang/String;)V

    throw v0
.end method

.method public setRequestProperty(Ljava/lang/String;Ljava/lang/String;)V
    .registers 5
    .param p1, "key"    # Ljava/lang/String;
    .param p2, "value"    # Ljava/lang/String;

    .line 489
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.setRequestProperty()"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 490
    new-instance v0, Ljava/lang/RuntimeException;

    const-string v1, "This class can only be used with alreadyretrieved data"

    invoke-direct {v0, v1}, Ljava/lang/RuntimeException;-><init>(Ljava/lang/String;)V

    throw v0
.end method

.method public setUseCaches(Z)V
    .registers 4
    .param p1, "isUsingCaches"    # Z

    .line 432
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.setUseCaches(boolean)"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 433
    new-instance v0, Ljava/lang/RuntimeException;

    const-string v1, "This class can only be used with alreadyretrieved data"

    invoke-direct {v0, v1}, Ljava/lang/RuntimeException;-><init>(Ljava/lang/String;)V

    throw v0
.end method

.method public usingProxy()Z
    .registers 3

    .line 167
    sget-object v0, Lorg/apache/commons/httpclient/util/HttpURLConnection;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter HttpURLConnection.usingProxy()"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 168
    new-instance v0, Ljava/lang/RuntimeException;

    const-string v1, "Not implemented yet"

    invoke-direct {v0, v1}, Ljava/lang/RuntimeException;-><init>(Ljava/lang/String;)V

    throw v0
.end method
