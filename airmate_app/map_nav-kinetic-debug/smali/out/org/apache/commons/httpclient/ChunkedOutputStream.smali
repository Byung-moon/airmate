.class public Lorg/apache/commons/httpclient/ChunkedOutputStream;
.super Ljava/io/OutputStream;
.source "ChunkedOutputStream.java"


# static fields
.field private static final CRLF:[B

.field private static final ENDCHUNK:[B

.field private static final ZERO:[B


# instance fields
.field private cache:[B

.field private cachePosition:I

.field private stream:Ljava/io/OutputStream;

.field private wroteLastChunk:Z


# direct methods
.method static constructor <clinit>()V
    .registers 3

    .line 47
    const/4 v0, 0x2

    new-array v0, v0, [B

    fill-array-data v0, :array_18

    sput-object v0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->CRLF:[B

    .line 50
    sget-object v0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->CRLF:[B

    sput-object v0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->ENDCHUNK:[B

    .line 53
    const/4 v0, 0x1

    new-array v0, v0, [B

    const/4 v1, 0x0

    const/16 v2, 0x30

    aput-byte v2, v0, v1

    sput-object v0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->ZERO:[B

    return-void

    nop

    :array_18
    .array-data 1
        0xdt
        0xat
    .end array-data
.end method

.method public constructor <init>(Ljava/io/OutputStream;)V
    .registers 3
    .param p1, "stream"    # Ljava/io/OutputStream;
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 85
    const/16 v0, 0x800

    invoke-direct {p0, p1, v0}, Lorg/apache/commons/httpclient/ChunkedOutputStream;-><init>(Ljava/io/OutputStream;I)V

    .line 86
    return-void
.end method

.method public constructor <init>(Ljava/io/OutputStream;I)V
    .registers 4
    .param p1, "stream"    # Ljava/io/OutputStream;
    .param p2, "bufferSize"    # I
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 73
    invoke-direct {p0}, Ljava/io/OutputStream;-><init>()V

    .line 56
    const/4 v0, 0x0

    iput-object v0, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->stream:Ljava/io/OutputStream;

    .line 60
    const/4 v0, 0x0

    iput v0, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->cachePosition:I

    .line 62
    iput-boolean v0, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->wroteLastChunk:Z

    .line 74
    new-array v0, p2, [B

    iput-object v0, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->cache:[B

    .line 75
    iput-object p1, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->stream:Ljava/io/OutputStream;

    .line 76
    return-void
.end method


# virtual methods
.method public close()V
    .registers 1
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 199
    invoke-virtual {p0}, Lorg/apache/commons/httpclient/ChunkedOutputStream;->finish()V

    .line 200
    invoke-super {p0}, Ljava/io/OutputStream;->close()V

    .line 201
    return-void
.end method

.method public finish()V
    .registers 2
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 142
    iget-boolean v0, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->wroteLastChunk:Z

    if-nez v0, :cond_d

    .line 143
    invoke-virtual {p0}, Lorg/apache/commons/httpclient/ChunkedOutputStream;->flushCache()V

    .line 144
    invoke-virtual {p0}, Lorg/apache/commons/httpclient/ChunkedOutputStream;->writeClosingChunk()V

    .line 145
    const/4 v0, 0x1

    iput-boolean v0, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->wroteLastChunk:Z

    .line 147
    :cond_d
    return-void
.end method

.method public flush()V
    .registers 2
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 191
    iget-object v0, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->stream:Ljava/io/OutputStream;

    invoke-virtual {v0}, Ljava/io/OutputStream;->flush()V

    .line 192
    return-void
.end method

.method protected flushCache()V
    .registers 6
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 96
    iget v0, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->cachePosition:I

    if-lez v0, :cond_3b

    .line 97
    new-instance v0, Ljava/lang/StringBuffer;

    invoke-direct {v0}, Ljava/lang/StringBuffer;-><init>()V

    iget v1, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->cachePosition:I

    invoke-static {v1}, Ljava/lang/Integer;->toHexString(I)Ljava/lang/String;

    move-result-object v1

    invoke-virtual {v0, v1}, Ljava/lang/StringBuffer;->append(Ljava/lang/String;)Ljava/lang/StringBuffer;

    const-string v1, "\r\n"

    invoke-virtual {v0, v1}, Ljava/lang/StringBuffer;->append(Ljava/lang/String;)Ljava/lang/StringBuffer;

    invoke-virtual {v0}, Ljava/lang/StringBuffer;->toString()Ljava/lang/String;

    move-result-object v0

    invoke-static {v0}, Lorg/apache/commons/httpclient/util/EncodingUtil;->getAsciiBytes(Ljava/lang/String;)[B

    move-result-object v0

    .line 99
    .local v0, "chunkHeader":[B
    iget-object v1, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->stream:Ljava/io/OutputStream;

    array-length v2, v0

    const/4 v3, 0x0

    invoke-virtual {v1, v0, v3, v2}, Ljava/io/OutputStream;->write([BII)V

    .line 100
    iget-object v1, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->stream:Ljava/io/OutputStream;

    iget-object v2, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->cache:[B

    iget v4, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->cachePosition:I

    invoke-virtual {v1, v2, v3, v4}, Ljava/io/OutputStream;->write([BII)V

    .line 101
    iget-object v1, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->stream:Ljava/io/OutputStream;

    sget-object v2, Lorg/apache/commons/httpclient/ChunkedOutputStream;->ENDCHUNK:[B

    sget-object v4, Lorg/apache/commons/httpclient/ChunkedOutputStream;->ENDCHUNK:[B

    array-length v4, v4

    invoke-virtual {v1, v2, v3, v4}, Ljava/io/OutputStream;->write([BII)V

    .line 102
    iput v3, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->cachePosition:I

    .line 104
    .end local v0    # "chunkHeader":[B
    :cond_3b
    return-void
.end method

.method protected flushCacheWithAppend([BII)V
    .registers 9
    .param p1, "bufferToAppend"    # [B
    .param p2, "off"    # I
    .param p3, "len"    # I
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 117
    new-instance v0, Ljava/lang/StringBuffer;

    invoke-direct {v0}, Ljava/lang/StringBuffer;-><init>()V

    iget v1, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->cachePosition:I

    add-int/2addr v1, p3

    invoke-static {v1}, Ljava/lang/Integer;->toHexString(I)Ljava/lang/String;

    move-result-object v1

    invoke-virtual {v0, v1}, Ljava/lang/StringBuffer;->append(Ljava/lang/String;)Ljava/lang/StringBuffer;

    const-string v1, "\r\n"

    invoke-virtual {v0, v1}, Ljava/lang/StringBuffer;->append(Ljava/lang/String;)Ljava/lang/StringBuffer;

    invoke-virtual {v0}, Ljava/lang/StringBuffer;->toString()Ljava/lang/String;

    move-result-object v0

    invoke-static {v0}, Lorg/apache/commons/httpclient/util/EncodingUtil;->getAsciiBytes(Ljava/lang/String;)[B

    move-result-object v0

    .line 119
    .local v0, "chunkHeader":[B
    iget-object v1, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->stream:Ljava/io/OutputStream;

    array-length v2, v0

    const/4 v3, 0x0

    invoke-virtual {v1, v0, v3, v2}, Ljava/io/OutputStream;->write([BII)V

    .line 120
    iget-object v1, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->stream:Ljava/io/OutputStream;

    iget-object v2, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->cache:[B

    iget v4, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->cachePosition:I

    invoke-virtual {v1, v2, v3, v4}, Ljava/io/OutputStream;->write([BII)V

    .line 121
    iget-object v1, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->stream:Ljava/io/OutputStream;

    invoke-virtual {v1, p1, p2, p3}, Ljava/io/OutputStream;->write([BII)V

    .line 122
    iget-object v1, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->stream:Ljava/io/OutputStream;

    sget-object v2, Lorg/apache/commons/httpclient/ChunkedOutputStream;->ENDCHUNK:[B

    sget-object v4, Lorg/apache/commons/httpclient/ChunkedOutputStream;->ENDCHUNK:[B

    array-length v4, v4

    invoke-virtual {v1, v2, v3, v4}, Ljava/io/OutputStream;->write([BII)V

    .line 123
    iput v3, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->cachePosition:I

    .line 124
    return-void
.end method

.method public write(I)V
    .registers 5
    .param p1, "b"    # I
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 160
    iget-object v0, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->cache:[B

    iget v1, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->cachePosition:I

    int-to-byte v2, p1

    aput-byte v2, v0, v1

    .line 161
    iget v0, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->cachePosition:I

    add-int/lit8 v0, v0, 0x1

    iput v0, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->cachePosition:I

    .line 162
    iget v0, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->cachePosition:I

    iget-object v1, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->cache:[B

    array-length v1, v1

    if-ne v0, v1, :cond_17

    invoke-virtual {p0}, Lorg/apache/commons/httpclient/ChunkedOutputStream;->flushCache()V

    .line 163
    :cond_17
    return-void
.end method

.method public write([B)V
    .registers 4
    .param p1, "b"    # [B
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 174
    array-length v0, p1

    const/4 v1, 0x0

    invoke-virtual {p0, p1, v1, v0}, Lorg/apache/commons/httpclient/ChunkedOutputStream;->write([BII)V

    .line 175
    return-void
.end method

.method public write([BII)V
    .registers 6
    .param p1, "src"    # [B
    .param p2, "off"    # I
    .param p3, "len"    # I
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 178
    iget-object v0, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->cache:[B

    array-length v0, v0

    iget v1, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->cachePosition:I

    sub-int/2addr v0, v1

    if-lt p3, v0, :cond_c

    .line 179
    invoke-virtual {p0, p1, p2, p3}, Lorg/apache/commons/httpclient/ChunkedOutputStream;->flushCacheWithAppend([BII)V

    goto :goto_18

    .line 181
    :cond_c
    iget-object v0, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->cache:[B

    iget v1, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->cachePosition:I

    invoke-static {p1, p2, v0, v1, p3}, Ljava/lang/System;->arraycopy(Ljava/lang/Object;ILjava/lang/Object;II)V

    .line 182
    iget v0, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->cachePosition:I

    add-int/2addr v0, p3

    iput v0, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->cachePosition:I

    .line 184
    :goto_18
    return-void
.end method

.method protected writeClosingChunk()V
    .registers 5
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 129
    iget-object v0, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->stream:Ljava/io/OutputStream;

    sget-object v1, Lorg/apache/commons/httpclient/ChunkedOutputStream;->ZERO:[B

    sget-object v2, Lorg/apache/commons/httpclient/ChunkedOutputStream;->ZERO:[B

    array-length v2, v2

    const/4 v3, 0x0

    invoke-virtual {v0, v1, v3, v2}, Ljava/io/OutputStream;->write([BII)V

    .line 130
    iget-object v0, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->stream:Ljava/io/OutputStream;

    sget-object v1, Lorg/apache/commons/httpclient/ChunkedOutputStream;->CRLF:[B

    sget-object v2, Lorg/apache/commons/httpclient/ChunkedOutputStream;->CRLF:[B

    array-length v2, v2

    invoke-virtual {v0, v1, v3, v2}, Ljava/io/OutputStream;->write([BII)V

    .line 131
    iget-object v0, p0, Lorg/apache/commons/httpclient/ChunkedOutputStream;->stream:Ljava/io/OutputStream;

    sget-object v1, Lorg/apache/commons/httpclient/ChunkedOutputStream;->ENDCHUNK:[B

    sget-object v2, Lorg/apache/commons/httpclient/ChunkedOutputStream;->ENDCHUNK:[B

    array-length v2, v2

    invoke-virtual {v0, v1, v3, v2}, Ljava/io/OutputStream;->write([BII)V

    .line 132
    return-void
.end method
