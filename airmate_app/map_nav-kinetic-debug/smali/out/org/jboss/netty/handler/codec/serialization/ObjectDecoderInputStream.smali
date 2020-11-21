.class public Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;
.super Ljava/io/InputStream;
.source "ObjectDecoderInputStream.java"

# interfaces
.implements Ljava/io/ObjectInput;


# instance fields
.field private final classResolver:Lorg/jboss/netty/handler/codec/serialization/ClassResolver;

.field private final in:Ljava/io/DataInputStream;

.field private final maxObjectSize:I


# direct methods
.method public constructor <init>(Ljava/io/InputStream;)V
    .registers 3
    .param p1, "in"    # Ljava/io/InputStream;

    .line 43
    const/4 v0, 0x0

    invoke-direct {p0, p1, v0}, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;-><init>(Ljava/io/InputStream;Ljava/lang/ClassLoader;)V

    .line 44
    return-void
.end method

.method public constructor <init>(Ljava/io/InputStream;I)V
    .registers 4
    .param p1, "in"    # Ljava/io/InputStream;
    .param p2, "maxObjectSize"    # I

    .line 72
    const/4 v0, 0x0

    invoke-direct {p0, p1, v0, p2}, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;-><init>(Ljava/io/InputStream;Ljava/lang/ClassLoader;I)V

    .line 73
    return-void
.end method

.method public constructor <init>(Ljava/io/InputStream;Ljava/lang/ClassLoader;)V
    .registers 4
    .param p1, "in"    # Ljava/io/InputStream;
    .param p2, "classLoader"    # Ljava/lang/ClassLoader;

    .line 57
    const/high16 v0, 0x100000

    invoke-direct {p0, p1, p2, v0}, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;-><init>(Ljava/io/InputStream;Ljava/lang/ClassLoader;I)V

    .line 58
    return-void
.end method

.method public constructor <init>(Ljava/io/InputStream;Ljava/lang/ClassLoader;I)V
    .registers 7
    .param p1, "in"    # Ljava/io/InputStream;
    .param p2, "classLoader"    # Ljava/lang/ClassLoader;
    .param p3, "maxObjectSize"    # I

    .line 89
    invoke-direct {p0}, Ljava/io/InputStream;-><init>()V

    .line 90
    if-eqz p1, :cond_38

    .line 93
    if-lez p3, :cond_21

    .line 96
    instance-of v0, p1, Ljava/io/DataInputStream;

    if-eqz v0, :cond_11

    .line 97
    move-object v0, p1

    check-cast v0, Ljava/io/DataInputStream;

    iput-object v0, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->in:Ljava/io/DataInputStream;

    goto :goto_18

    .line 99
    :cond_11
    new-instance v0, Ljava/io/DataInputStream;

    invoke-direct {v0, p1}, Ljava/io/DataInputStream;-><init>(Ljava/io/InputStream;)V

    iput-object v0, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->in:Ljava/io/DataInputStream;

    .line 101
    :goto_18
    invoke-static {p2}, Lorg/jboss/netty/handler/codec/serialization/ClassResolvers;->weakCachingResolver(Ljava/lang/ClassLoader;)Lorg/jboss/netty/handler/codec/serialization/ClassResolver;

    move-result-object v0

    iput-object v0, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->classResolver:Lorg/jboss/netty/handler/codec/serialization/ClassResolver;

    .line 102
    iput p3, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->maxObjectSize:I

    .line 103
    return-void

    .line 94
    :cond_21
    new-instance v0, Ljava/lang/IllegalArgumentException;

    new-instance v1, Ljava/lang/StringBuilder;

    invoke-direct {v1}, Ljava/lang/StringBuilder;-><init>()V

    const-string v2, "maxObjectSize: "

    invoke-virtual {v1, v2}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v1, p3}, Ljava/lang/StringBuilder;->append(I)Ljava/lang/StringBuilder;

    invoke-virtual {v1}, Ljava/lang/StringBuilder;->toString()Ljava/lang/String;

    move-result-object v1

    invoke-direct {v0, v1}, Ljava/lang/IllegalArgumentException;-><init>(Ljava/lang/String;)V

    throw v0

    .line 91
    :cond_38
    new-instance v0, Ljava/lang/NullPointerException;

    const-string v1, "in"

    invoke-direct {v0, v1}, Ljava/lang/NullPointerException;-><init>(Ljava/lang/String;)V

    throw v0
.end method


# virtual methods
.method public available()I
    .registers 2
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 120
    iget-object v0, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->in:Ljava/io/DataInputStream;

    invoke-virtual {v0}, Ljava/io/DataInputStream;->available()I

    move-result v0

    return v0
.end method

.method public close()V
    .registers 2
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 125
    iget-object v0, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->in:Ljava/io/DataInputStream;

    invoke-virtual {v0}, Ljava/io/DataInputStream;->close()V

    .line 126
    return-void
.end method

.method public mark(I)V
    .registers 3
    .param p1, "readlimit"    # I

    .line 130
    iget-object v0, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->in:Ljava/io/DataInputStream;

    invoke-virtual {v0, p1}, Ljava/io/DataInputStream;->mark(I)V

    .line 131
    return-void
.end method

.method public markSupported()Z
    .registers 2

    .line 135
    iget-object v0, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->in:Ljava/io/DataInputStream;

    invoke-virtual {v0}, Ljava/io/DataInputStream;->markSupported()Z

    move-result v0

    return v0
.end method

.method public read()I
    .registers 2
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 140
    iget-object v0, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->in:Ljava/io/DataInputStream;

    invoke-virtual {v0}, Ljava/io/DataInputStream;->read()I

    move-result v0

    return v0
.end method

.method public final read([B)I
    .registers 3
    .param p1, "b"    # [B
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 150
    iget-object v0, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->in:Ljava/io/DataInputStream;

    invoke-virtual {v0, p1}, Ljava/io/DataInputStream;->read([B)I

    move-result v0

    return v0
.end method

.method public final read([BII)I
    .registers 5
    .param p1, "b"    # [B
    .param p2, "off"    # I
    .param p3, "len"    # I
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 145
    iget-object v0, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->in:Ljava/io/DataInputStream;

    invoke-virtual {v0, p1, p2, p3}, Ljava/io/DataInputStream;->read([BII)I

    move-result v0

    return v0
.end method

.method public final readBoolean()Z
    .registers 2
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 154
    iget-object v0, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->in:Ljava/io/DataInputStream;

    invoke-virtual {v0}, Ljava/io/DataInputStream;->readBoolean()Z

    move-result v0

    return v0
.end method

.method public final readByte()B
    .registers 2
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 158
    iget-object v0, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->in:Ljava/io/DataInputStream;

    invoke-virtual {v0}, Ljava/io/DataInputStream;->readByte()B

    move-result v0

    return v0
.end method

.method public final readChar()C
    .registers 2
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 162
    iget-object v0, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->in:Ljava/io/DataInputStream;

    invoke-virtual {v0}, Ljava/io/DataInputStream;->readChar()C

    move-result v0

    return v0
.end method

.method public final readDouble()D
    .registers 3
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 166
    iget-object v0, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->in:Ljava/io/DataInputStream;

    invoke-virtual {v0}, Ljava/io/DataInputStream;->readDouble()D

    move-result-wide v0

    return-wide v0
.end method

.method public final readFloat()F
    .registers 2
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 170
    iget-object v0, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->in:Ljava/io/DataInputStream;

    invoke-virtual {v0}, Ljava/io/DataInputStream;->readFloat()F

    move-result v0

    return v0
.end method

.method public final readFully([B)V
    .registers 3
    .param p1, "b"    # [B
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 178
    iget-object v0, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->in:Ljava/io/DataInputStream;

    invoke-virtual {v0, p1}, Ljava/io/DataInputStream;->readFully([B)V

    .line 179
    return-void
.end method

.method public final readFully([BII)V
    .registers 5
    .param p1, "b"    # [B
    .param p2, "off"    # I
    .param p3, "len"    # I
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 174
    iget-object v0, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->in:Ljava/io/DataInputStream;

    invoke-virtual {v0, p1, p2, p3}, Ljava/io/DataInputStream;->readFully([BII)V

    .line 175
    return-void
.end method

.method public final readInt()I
    .registers 2
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 182
    iget-object v0, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->in:Ljava/io/DataInputStream;

    invoke-virtual {v0}, Ljava/io/DataInputStream;->readInt()I

    move-result v0

    return v0
.end method

.method public final readLine()Ljava/lang/String;
    .registers 2
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .annotation runtime Ljava/lang/Deprecated;
    .end annotation

    .line 187
    iget-object v0, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->in:Ljava/io/DataInputStream;

    invoke-virtual {v0}, Ljava/io/DataInputStream;->readLine()Ljava/lang/String;

    move-result-object v0

    return-object v0
.end method

.method public final readLong()J
    .registers 3
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 191
    iget-object v0, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->in:Ljava/io/DataInputStream;

    invoke-virtual {v0}, Ljava/io/DataInputStream;->readLong()J

    move-result-wide v0

    return-wide v0
.end method

.method public readObject()Ljava/lang/Object;
    .registers 5
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/lang/ClassNotFoundException;,
            Ljava/io/IOException;
        }
    .end annotation

    .line 106
    invoke-virtual {p0}, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->readInt()I

    move-result v0

    .line 107
    .local v0, "dataLen":I
    if-lez v0, :cond_3e

    .line 110
    iget v1, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->maxObjectSize:I

    if-gt v0, v1, :cond_18

    .line 115
    new-instance v1, Lorg/jboss/netty/handler/codec/serialization/CompactObjectInputStream;

    iget-object v2, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->in:Ljava/io/DataInputStream;

    iget-object v3, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->classResolver:Lorg/jboss/netty/handler/codec/serialization/ClassResolver;

    invoke-direct {v1, v2, v3}, Lorg/jboss/netty/handler/codec/serialization/CompactObjectInputStream;-><init>(Ljava/io/InputStream;Lorg/jboss/netty/handler/codec/serialization/ClassResolver;)V

    invoke-virtual {v1}, Lorg/jboss/netty/handler/codec/serialization/CompactObjectInputStream;->readObject()Ljava/lang/Object;

    move-result-object v1

    return-object v1

    .line 111
    :cond_18
    new-instance v1, Ljava/io/StreamCorruptedException;

    new-instance v2, Ljava/lang/StringBuilder;

    invoke-direct {v2}, Ljava/lang/StringBuilder;-><init>()V

    const-string v3, "data length too big: "

    invoke-virtual {v2, v3}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v2, v0}, Ljava/lang/StringBuilder;->append(I)Ljava/lang/StringBuilder;

    const-string v3, " (max: "

    invoke-virtual {v2, v3}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    iget v3, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->maxObjectSize:I

    invoke-virtual {v2, v3}, Ljava/lang/StringBuilder;->append(I)Ljava/lang/StringBuilder;

    const/16 v3, 0x29

    invoke-virtual {v2, v3}, Ljava/lang/StringBuilder;->append(C)Ljava/lang/StringBuilder;

    invoke-virtual {v2}, Ljava/lang/StringBuilder;->toString()Ljava/lang/String;

    move-result-object v2

    invoke-direct {v1, v2}, Ljava/io/StreamCorruptedException;-><init>(Ljava/lang/String;)V

    throw v1

    .line 108
    :cond_3e
    new-instance v1, Ljava/io/StreamCorruptedException;

    new-instance v2, Ljava/lang/StringBuilder;

    invoke-direct {v2}, Ljava/lang/StringBuilder;-><init>()V

    const-string v3, "invalid data length: "

    invoke-virtual {v2, v3}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v2, v0}, Ljava/lang/StringBuilder;->append(I)Ljava/lang/StringBuilder;

    invoke-virtual {v2}, Ljava/lang/StringBuilder;->toString()Ljava/lang/String;

    move-result-object v2

    invoke-direct {v1, v2}, Ljava/io/StreamCorruptedException;-><init>(Ljava/lang/String;)V

    throw v1
.end method

.method public final readShort()S
    .registers 2
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 195
    iget-object v0, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->in:Ljava/io/DataInputStream;

    invoke-virtual {v0}, Ljava/io/DataInputStream;->readShort()S

    move-result v0

    return v0
.end method

.method public final readUTF()Ljava/lang/String;
    .registers 2
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 207
    iget-object v0, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->in:Ljava/io/DataInputStream;

    invoke-virtual {v0}, Ljava/io/DataInputStream;->readUTF()Ljava/lang/String;

    move-result-object v0

    return-object v0
.end method

.method public final readUnsignedByte()I
    .registers 2
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 199
    iget-object v0, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->in:Ljava/io/DataInputStream;

    invoke-virtual {v0}, Ljava/io/DataInputStream;->readUnsignedByte()I

    move-result v0

    return v0
.end method

.method public final readUnsignedShort()I
    .registers 2
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 203
    iget-object v0, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->in:Ljava/io/DataInputStream;

    invoke-virtual {v0}, Ljava/io/DataInputStream;->readUnsignedShort()I

    move-result v0

    return v0
.end method

.method public reset()V
    .registers 2
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 212
    iget-object v0, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->in:Ljava/io/DataInputStream;

    invoke-virtual {v0}, Ljava/io/DataInputStream;->reset()V

    .line 213
    return-void
.end method

.method public skip(J)J
    .registers 5
    .param p1, "n"    # J
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 217
    iget-object v0, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->in:Ljava/io/DataInputStream;

    invoke-virtual {v0, p1, p2}, Ljava/io/DataInputStream;->skip(J)J

    move-result-wide v0

    return-wide v0
.end method

.method public final skipBytes(I)I
    .registers 3
    .param p1, "n"    # I
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 221
    iget-object v0, p0, Lorg/jboss/netty/handler/codec/serialization/ObjectDecoderInputStream;->in:Ljava/io/DataInputStream;

    invoke-virtual {v0, p1}, Ljava/io/DataInputStream;->skipBytes(I)I

    move-result v0

    return v0
.end method
