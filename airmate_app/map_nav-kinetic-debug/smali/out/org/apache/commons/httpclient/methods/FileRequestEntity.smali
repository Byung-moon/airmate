.class public Lorg/apache/commons/httpclient/methods/FileRequestEntity;
.super Ljava/lang/Object;
.source "FileRequestEntity.java"

# interfaces
.implements Lorg/apache/commons/httpclient/methods/RequestEntity;


# instance fields
.field final contentType:Ljava/lang/String;

.field final file:Ljava/io/File;


# direct methods
.method public constructor <init>(Ljava/io/File;Ljava/lang/String;)V
    .registers 5
    .param p1, "file"    # Ljava/io/File;
    .param p2, "contentType"    # Ljava/lang/String;

    .line 51
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    .line 52
    if-eqz p1, :cond_a

    .line 55
    iput-object p1, p0, Lorg/apache/commons/httpclient/methods/FileRequestEntity;->file:Ljava/io/File;

    .line 56
    iput-object p2, p0, Lorg/apache/commons/httpclient/methods/FileRequestEntity;->contentType:Ljava/lang/String;

    .line 57
    return-void

    .line 53
    :cond_a
    new-instance v0, Ljava/lang/IllegalArgumentException;

    const-string v1, "File may not be null"

    invoke-direct {v0, v1}, Ljava/lang/IllegalArgumentException;-><init>(Ljava/lang/String;)V

    throw v0
.end method


# virtual methods
.method public getContentLength()J
    .registers 3

    .line 59
    iget-object v0, p0, Lorg/apache/commons/httpclient/methods/FileRequestEntity;->file:Ljava/io/File;

    invoke-virtual {v0}, Ljava/io/File;->length()J

    move-result-wide v0

    return-wide v0
.end method

.method public getContentType()Ljava/lang/String;
    .registers 2

    .line 63
    iget-object v0, p0, Lorg/apache/commons/httpclient/methods/FileRequestEntity;->contentType:Ljava/lang/String;

    return-object v0
.end method

.method public isRepeatable()Z
    .registers 2

    .line 67
    const/4 v0, 0x1

    return v0
.end method

.method public writeRequest(Ljava/io/OutputStream;)V
    .registers 6
    .param p1, "out"    # Ljava/io/OutputStream;
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 71
    const/16 v0, 0x1000

    new-array v0, v0, [B

    .line 72
    .local v0, "tmp":[B
    const/4 v1, 0x0

    .line 73
    .local v1, "i":I
    new-instance v2, Ljava/io/FileInputStream;

    iget-object v3, p0, Lorg/apache/commons/httpclient/methods/FileRequestEntity;->file:Ljava/io/File;

    invoke-direct {v2, v3}, Ljava/io/FileInputStream;-><init>(Ljava/io/File;)V

    .line 75
    .local v2, "instream":Ljava/io/InputStream;
    :goto_c
    :try_start_c
    invoke-virtual {v2, v0}, Ljava/io/InputStream;->read([B)I

    move-result v3

    move v1, v3

    if-ltz v3, :cond_18

    .line 76
    const/4 v3, 0x0

    invoke-virtual {p1, v0, v3, v1}, Ljava/io/OutputStream;->write([BII)V
    :try_end_17
    .catchall {:try_start_c .. :try_end_17} :catchall_1d

    goto :goto_c

    .line 79
    :cond_18
    invoke-virtual {v2}, Ljava/io/InputStream;->close()V

    .line 80
    nop

    .line 81
    return-void

    .line 79
    :catchall_1d
    move-exception v3

    invoke-virtual {v2}, Ljava/io/InputStream;->close()V

    throw v3
.end method
