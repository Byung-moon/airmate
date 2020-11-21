.class public Lorg/xbill/DNS/NULLRecord;
.super Lorg/xbill/DNS/Record;
.source "NULLRecord.java"


# static fields
.field private static final serialVersionUID:J = -0x507147d6c9b5549aL


# instance fields
.field private data:[B


# direct methods
.method constructor <init>()V
    .registers 1

    .line 20
    invoke-direct {p0}, Lorg/xbill/DNS/Record;-><init>()V

    return-void
.end method

.method public constructor <init>(Lorg/xbill/DNS/Name;IJ[B)V
    .registers 12
    .param p1, "name"    # Lorg/xbill/DNS/Name;
    .param p2, "dclass"    # I
    .param p3, "ttl"    # J
    .param p5, "data"    # [B

    .line 33
    const/16 v2, 0xa

    move-object v0, p0

    move-object v1, p1

    move v3, p2

    move-wide v4, p3

    invoke-direct/range {v0 .. v5}, Lorg/xbill/DNS/Record;-><init>(Lorg/xbill/DNS/Name;IIJ)V

    .line 35
    array-length v0, p5

    const v1, 0xffff

    if-gt v0, v1, :cond_12

    .line 38
    iput-object p5, p0, Lorg/xbill/DNS/NULLRecord;->data:[B

    .line 39
    return-void

    .line 36
    :cond_12
    new-instance v0, Ljava/lang/IllegalArgumentException;

    const-string v1, "data must be <65536 bytes"

    invoke-direct {v0, v1}, Ljava/lang/IllegalArgumentException;-><init>(Ljava/lang/String;)V

    throw v0
.end method


# virtual methods
.method public getData()[B
    .registers 2

    .line 59
    iget-object v0, p0, Lorg/xbill/DNS/NULLRecord;->data:[B

    return-object v0
.end method

.method getObject()Lorg/xbill/DNS/Record;
    .registers 2

    .line 24
    new-instance v0, Lorg/xbill/DNS/NULLRecord;

    invoke-direct {v0}, Lorg/xbill/DNS/NULLRecord;-><init>()V

    return-object v0
.end method

.method rdataFromString(Lorg/xbill/DNS/Tokenizer;Lorg/xbill/DNS/Name;)V
    .registers 4
    .param p1, "st"    # Lorg/xbill/DNS/Tokenizer;
    .param p2, "origin"    # Lorg/xbill/DNS/Name;
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 48
    const-string v0, "no defined text format for NULL records"

    invoke-virtual {p1, v0}, Lorg/xbill/DNS/Tokenizer;->exception(Ljava/lang/String;)Lorg/xbill/DNS/TextParseException;

    move-result-object v0

    throw v0
.end method

.method rrFromWire(Lorg/xbill/DNS/DNSInput;)V
    .registers 3
    .param p1, "in"    # Lorg/xbill/DNS/DNSInput;
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 43
    invoke-virtual {p1}, Lorg/xbill/DNS/DNSInput;->readByteArray()[B

    move-result-object v0

    iput-object v0, p0, Lorg/xbill/DNS/NULLRecord;->data:[B

    .line 44
    return-void
.end method

.method rrToString()Ljava/lang/String;
    .registers 2

    .line 53
    iget-object v0, p0, Lorg/xbill/DNS/NULLRecord;->data:[B

    invoke-static {v0}, Lorg/xbill/DNS/NULLRecord;->unknownToString([B)Ljava/lang/String;

    move-result-object v0

    return-object v0
.end method

.method rrToWire(Lorg/xbill/DNS/DNSOutput;Lorg/xbill/DNS/Compression;Z)V
    .registers 5
    .param p1, "out"    # Lorg/xbill/DNS/DNSOutput;
    .param p2, "c"    # Lorg/xbill/DNS/Compression;
    .param p3, "canonical"    # Z

    .line 64
    iget-object v0, p0, Lorg/xbill/DNS/NULLRecord;->data:[B

    invoke-virtual {p1, v0}, Lorg/xbill/DNS/DNSOutput;->writeByteArray([B)V

    .line 65
    return-void
.end method
