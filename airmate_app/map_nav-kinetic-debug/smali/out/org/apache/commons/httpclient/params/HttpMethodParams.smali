.class public Lorg/apache/commons/httpclient/params/HttpMethodParams;
.super Lorg/apache/commons/httpclient/params/DefaultHttpParams;
.source "HttpMethodParams.java"


# static fields
.field public static final BUFFER_WARN_TRIGGER_LIMIT:Ljava/lang/String; = "http.method.response.buffer.warnlimit"

.field public static final COOKIE_POLICY:Ljava/lang/String; = "http.protocol.cookie-policy"

.field public static final CREDENTIAL_CHARSET:Ljava/lang/String; = "http.protocol.credential-charset"

.field public static final DATE_PATTERNS:Ljava/lang/String; = "http.dateparser.patterns"

.field public static final HEAD_BODY_CHECK_TIMEOUT:Ljava/lang/String; = "http.protocol.head-body-timeout"

.field public static final HTTP_CONTENT_CHARSET:Ljava/lang/String; = "http.protocol.content-charset"

.field public static final HTTP_ELEMENT_CHARSET:Ljava/lang/String; = "http.protocol.element-charset"

.field public static final HTTP_URI_CHARSET:Ljava/lang/String; = "http.protocol.uri-charset"

.field private static final LOG:Lorg/apache/commons/logging/Log;

.field public static final MULTIPART_BOUNDARY:Ljava/lang/String; = "http.method.multipart.boundary"

.field private static final PROTOCOL_STRICTNESS_PARAMETERS:[Ljava/lang/String;

.field public static final PROTOCOL_VERSION:Ljava/lang/String; = "http.protocol.version"

.field public static final REJECT_HEAD_BODY:Ljava/lang/String; = "http.protocol.reject-head-body"

.field public static final RETRY_HANDLER:Ljava/lang/String; = "http.method.retry-handler"

.field public static final SINGLE_COOKIE_HEADER:Ljava/lang/String; = "http.protocol.single-cookie-header"

.field public static final SO_TIMEOUT:Ljava/lang/String; = "http.socket.timeout"

.field public static final STATUS_LINE_GARBAGE_LIMIT:Ljava/lang/String; = "http.protocol.status-line-garbage-limit"

.field public static final STRICT_TRANSFER_ENCODING:Ljava/lang/String; = "http.protocol.strict-transfer-encoding"

.field public static final UNAMBIGUOUS_STATUS_LINE:Ljava/lang/String; = "http.protocol.unambiguous-statusline"

.field public static final USER_AGENT:Ljava/lang/String; = "http.useragent"

.field public static final USE_EXPECT_CONTINUE:Ljava/lang/String; = "http.protocol.expect-continue"

.field public static final VIRTUAL_HOST:Ljava/lang/String; = "http.virtual-host"

.field public static final WARN_EXTRA_INPUT:Ljava/lang/String; = "http.protocol.warn-extra-input"

.field static synthetic class$org$apache$commons$httpclient$params$HttpMethodParams:Ljava/lang/Class;


# direct methods
.method static constructor <clinit>()V
    .registers 3

    .line 55
    sget-object v0, Lorg/apache/commons/httpclient/params/HttpMethodParams;->class$org$apache$commons$httpclient$params$HttpMethodParams:Ljava/lang/Class;

    if-nez v0, :cond_d

    const-string v0, "org.apache.commons.httpclient.params.HttpMethodParams"

    invoke-static {v0}, Lorg/apache/commons/httpclient/params/HttpMethodParams;->class$(Ljava/lang/String;)Ljava/lang/Class;

    move-result-object v0

    sput-object v0, Lorg/apache/commons/httpclient/params/HttpMethodParams;->class$org$apache$commons$httpclient$params$HttpMethodParams:Ljava/lang/Class;

    goto :goto_f

    :cond_d
    sget-object v0, Lorg/apache/commons/httpclient/params/HttpMethodParams;->class$org$apache$commons$httpclient$params$HttpMethodParams:Ljava/lang/Class;

    :goto_f
    invoke-static {v0}, Lorg/apache/commons/logging/LogFactory;->getLog(Ljava/lang/Class;)Lorg/apache/commons/logging/Log;

    move-result-object v0

    sput-object v0, Lorg/apache/commons/httpclient/params/HttpMethodParams;->LOG:Lorg/apache/commons/logging/Log;

    .line 491
    const/4 v0, 0x5

    new-array v0, v0, [Ljava/lang/String;

    const/4 v1, 0x0

    const-string v2, "http.protocol.unambiguous-statusline"

    aput-object v2, v0, v1

    const/4 v1, 0x1

    const-string v2, "http.protocol.single-cookie-header"

    aput-object v2, v0, v1

    const/4 v1, 0x2

    const-string v2, "http.protocol.strict-transfer-encoding"

    aput-object v2, v0, v1

    const/4 v1, 0x3

    const-string v2, "http.protocol.reject-head-body"

    aput-object v2, v0, v1

    const/4 v1, 0x4

    const-string v2, "http.protocol.warn-extra-input"

    aput-object v2, v0, v1

    sput-object v0, Lorg/apache/commons/httpclient/params/HttpMethodParams;->PROTOCOL_STRICTNESS_PARAMETERS:[Ljava/lang/String;

    return-void
.end method

.method public constructor <init>()V
    .registers 2

    .line 294
    invoke-static {}, Lorg/apache/commons/httpclient/params/HttpMethodParams;->getDefaultParams()Lorg/apache/commons/httpclient/params/HttpParams;

    move-result-object v0

    invoke-direct {p0, v0}, Lorg/apache/commons/httpclient/params/DefaultHttpParams;-><init>(Lorg/apache/commons/httpclient/params/HttpParams;)V

    .line 295
    return-void
.end method

.method public constructor <init>(Lorg/apache/commons/httpclient/params/HttpParams;)V
    .registers 2
    .param p1, "defaults"    # Lorg/apache/commons/httpclient/params/HttpParams;

    .line 309
    invoke-direct {p0, p1}, Lorg/apache/commons/httpclient/params/DefaultHttpParams;-><init>(Lorg/apache/commons/httpclient/params/HttpParams;)V

    .line 310
    return-void
.end method

.method static synthetic class$(Ljava/lang/String;)Ljava/lang/Class;
    .registers 4
    .param p0, "x0"    # Ljava/lang/String;

    .line 55
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
.method public getContentCharset()Ljava/lang/String;
    .registers 4

    .line 339
    const-string v0, "http.protocol.content-charset"

    invoke-virtual {p0, v0}, Lorg/apache/commons/httpclient/params/HttpMethodParams;->getParameter(Ljava/lang/String;)Ljava/lang/Object;

    move-result-object v0

    check-cast v0, Ljava/lang/String;

    .line 340
    .local v0, "charset":Ljava/lang/String;
    if-nez v0, :cond_13

    .line 341
    sget-object v1, Lorg/apache/commons/httpclient/params/HttpMethodParams;->LOG:Lorg/apache/commons/logging/Log;

    const-string v2, "Default content charset not configured, using ISO-8859-1"

    invoke-interface {v1, v2}, Lorg/apache/commons/logging/Log;->warn(Ljava/lang/Object;)V

    .line 342
    const-string v0, "ISO-8859-1"

    .line 344
    :cond_13
    return-object v0
.end method

.method public getCookiePolicy()Ljava/lang/String;
    .registers 3

    .line 433
    const-string v0, "http.protocol.cookie-policy"

    invoke-virtual {p0, v0}, Lorg/apache/commons/httpclient/params/HttpMethodParams;->getParameter(Ljava/lang/String;)Ljava/lang/Object;

    move-result-object v0

    .line 434
    .local v0, "param":Ljava/lang/Object;
    if-nez v0, :cond_b

    .line 435
    const-string v1, "default"

    return-object v1

    .line 437
    :cond_b
    move-object v1, v0

    check-cast v1, Ljava/lang/String;

    return-object v1
.end method

.method public getCredentialCharset()Ljava/lang/String;
    .registers 4

    .line 382
    const-string v0, "http.protocol.credential-charset"

    invoke-virtual {p0, v0}, Lorg/apache/commons/httpclient/params/HttpMethodParams;->getParameter(Ljava/lang/String;)Ljava/lang/Object;

    move-result-object v0

    check-cast v0, Ljava/lang/String;

    .line 383
    .local v0, "charset":Ljava/lang/String;
    if-nez v0, :cond_15

    .line 384
    sget-object v1, Lorg/apache/commons/httpclient/params/HttpMethodParams;->LOG:Lorg/apache/commons/logging/Log;

    const-string v2, "Credential charset not configured, using HTTP element charset"

    invoke-interface {v1, v2}, Lorg/apache/commons/logging/Log;->debug(Ljava/lang/Object;)V

    .line 385
    invoke-virtual {p0}, Lorg/apache/commons/httpclient/params/HttpMethodParams;->getHttpElementCharset()Ljava/lang/String;

    move-result-object v0

    .line 387
    :cond_15
    return-object v0
.end method

.method public getHttpElementCharset()Ljava/lang/String;
    .registers 4

    .line 317
    const-string v0, "http.protocol.element-charset"

    invoke-virtual {p0, v0}, Lorg/apache/commons/httpclient/params/HttpMethodParams;->getParameter(Ljava/lang/String;)Ljava/lang/Object;

    move-result-object v0

    check-cast v0, Ljava/lang/String;

    .line 318
    .local v0, "charset":Ljava/lang/String;
    if-nez v0, :cond_13

    .line 319
    sget-object v1, Lorg/apache/commons/httpclient/params/HttpMethodParams;->LOG:Lorg/apache/commons/logging/Log;

    const-string v2, "HTTP element charset not configured, using US-ASCII"

    invoke-interface {v1, v2}, Lorg/apache/commons/logging/Log;->warn(Ljava/lang/Object;)V

    .line 320
    const-string v0, "US-ASCII"

    .line 322
    :cond_13
    return-object v0
.end method

.method public getSoTimeout()I
    .registers 3

    .line 459
    const-string v0, "http.socket.timeout"

    const/4 v1, 0x0

    invoke-virtual {p0, v0, v1}, Lorg/apache/commons/httpclient/params/HttpMethodParams;->getIntParameter(Ljava/lang/String;I)I

    move-result v0

    return v0
.end method

.method public getUriCharset()Ljava/lang/String;
    .registers 2

    .line 360
    const-string v0, "http.protocol.uri-charset"

    invoke-virtual {p0, v0}, Lorg/apache/commons/httpclient/params/HttpMethodParams;->getParameter(Ljava/lang/String;)Ljava/lang/Object;

    move-result-object v0

    check-cast v0, Ljava/lang/String;

    .line 361
    .local v0, "charset":Ljava/lang/String;
    if-nez v0, :cond_c

    .line 362
    const-string v0, "UTF-8"

    .line 364
    :cond_c
    return-object v0
.end method

.method public getVersion()Lorg/apache/commons/httpclient/HttpVersion;
    .registers 3

    .line 406
    const-string v0, "http.protocol.version"

    invoke-virtual {p0, v0}, Lorg/apache/commons/httpclient/params/HttpMethodParams;->getParameter(Ljava/lang/String;)Ljava/lang/Object;

    move-result-object v0

    .line 407
    .local v0, "param":Ljava/lang/Object;
    if-nez v0, :cond_b

    .line 408
    sget-object v1, Lorg/apache/commons/httpclient/HttpVersion;->HTTP_1_1:Lorg/apache/commons/httpclient/HttpVersion;

    return-object v1

    .line 410
    :cond_b
    move-object v1, v0

    check-cast v1, Lorg/apache/commons/httpclient/HttpVersion;

    return-object v1
.end method

.method public getVirtualHost()Ljava/lang/String;
    .registers 2

    .line 488
    const-string v0, "http.virtual-host"

    invoke-virtual {p0, v0}, Lorg/apache/commons/httpclient/params/HttpMethodParams;->getParameter(Ljava/lang/String;)Ljava/lang/Object;

    move-result-object v0

    check-cast v0, Ljava/lang/String;

    return-object v0
.end method

.method public makeLenient()V
    .registers 3

    .line 518
    sget-object v0, Lorg/apache/commons/httpclient/params/HttpMethodParams;->PROTOCOL_STRICTNESS_PARAMETERS:[Ljava/lang/String;

    sget-object v1, Ljava/lang/Boolean;->FALSE:Ljava/lang/Boolean;

    invoke-virtual {p0, v0, v1}, Lorg/apache/commons/httpclient/params/HttpMethodParams;->setParameters([Ljava/lang/String;Ljava/lang/Object;)V

    .line 519
    const-string v0, "http.protocol.status-line-garbage-limit"

    const v1, 0x7fffffff

    invoke-virtual {p0, v0, v1}, Lorg/apache/commons/httpclient/params/HttpMethodParams;->setIntParameter(Ljava/lang/String;I)V

    .line 520
    return-void
.end method

.method public makeStrict()V
    .registers 3

    .line 507
    sget-object v0, Lorg/apache/commons/httpclient/params/HttpMethodParams;->PROTOCOL_STRICTNESS_PARAMETERS:[Ljava/lang/String;

    sget-object v1, Ljava/lang/Boolean;->TRUE:Ljava/lang/Boolean;

    invoke-virtual {p0, v0, v1}, Lorg/apache/commons/httpclient/params/HttpMethodParams;->setParameters([Ljava/lang/String;Ljava/lang/Object;)V

    .line 508
    const-string v0, "http.protocol.status-line-garbage-limit"

    const/4 v1, 0x0

    invoke-virtual {p0, v0, v1}, Lorg/apache/commons/httpclient/params/HttpMethodParams;->setIntParameter(Ljava/lang/String;I)V

    .line 509
    return-void
.end method

.method public setContentCharset(Ljava/lang/String;)V
    .registers 3
    .param p1, "charset"    # Ljava/lang/String;

    .line 373
    const-string v0, "http.protocol.content-charset"

    invoke-virtual {p0, v0, p1}, Lorg/apache/commons/httpclient/params/HttpMethodParams;->setParameter(Ljava/lang/String;Ljava/lang/Object;)V

    .line 374
    return-void
.end method

.method public setCookiePolicy(Ljava/lang/String;)V
    .registers 3
    .param p1, "policy"    # Ljava/lang/String;

    .line 448
    const-string v0, "http.protocol.cookie-policy"

    invoke-virtual {p0, v0, p1}, Lorg/apache/commons/httpclient/params/HttpMethodParams;->setParameter(Ljava/lang/String;Ljava/lang/Object;)V

    .line 449
    return-void
.end method

.method public setCredentialCharset(Ljava/lang/String;)V
    .registers 3
    .param p1, "charset"    # Ljava/lang/String;

    .line 395
    const-string v0, "http.protocol.credential-charset"

    invoke-virtual {p0, v0, p1}, Lorg/apache/commons/httpclient/params/HttpMethodParams;->setParameter(Ljava/lang/String;Ljava/lang/Object;)V

    .line 396
    return-void
.end method

.method public setHttpElementCharset(Ljava/lang/String;)V
    .registers 3
    .param p1, "charset"    # Ljava/lang/String;

    .line 330
    const-string v0, "http.protocol.element-charset"

    invoke-virtual {p0, v0, p1}, Lorg/apache/commons/httpclient/params/HttpMethodParams;->setParameter(Ljava/lang/String;Ljava/lang/Object;)V

    .line 331
    return-void
.end method

.method public setSoTimeout(I)V
    .registers 3
    .param p1, "timeout"    # I

    .line 470
    const-string v0, "http.socket.timeout"

    invoke-virtual {p0, v0, p1}, Lorg/apache/commons/httpclient/params/HttpMethodParams;->setIntParameter(Ljava/lang/String;I)V

    .line 471
    return-void
.end method

.method public setUriCharset(Ljava/lang/String;)V
    .registers 3
    .param p1, "charset"    # Ljava/lang/String;

    .line 352
    const-string v0, "http.protocol.uri-charset"

    invoke-virtual {p0, v0, p1}, Lorg/apache/commons/httpclient/params/HttpMethodParams;->setParameter(Ljava/lang/String;Ljava/lang/Object;)V

    .line 353
    return-void
.end method

.method public setVersion(Lorg/apache/commons/httpclient/HttpVersion;)V
    .registers 3
    .param p1, "version"    # Lorg/apache/commons/httpclient/HttpVersion;

    .line 421
    const-string v0, "http.protocol.version"

    invoke-virtual {p0, v0, p1}, Lorg/apache/commons/httpclient/params/HttpMethodParams;->setParameter(Ljava/lang/String;Ljava/lang/Object;)V

    .line 422
    return-void
.end method

.method public setVirtualHost(Ljava/lang/String;)V
    .registers 3
    .param p1, "hostname"    # Ljava/lang/String;

    .line 479
    const-string v0, "http.virtual-host"

    invoke-virtual {p0, v0, p1}, Lorg/apache/commons/httpclient/params/HttpMethodParams;->setParameter(Ljava/lang/String;Ljava/lang/Object;)V

    .line 480
    return-void
.end method
