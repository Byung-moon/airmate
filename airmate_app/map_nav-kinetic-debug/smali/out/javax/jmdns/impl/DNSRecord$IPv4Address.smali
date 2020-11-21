.class public Ljavax/jmdns/impl/DNSRecord$IPv4Address;
.super Ljavax/jmdns/impl/DNSRecord$Address;
.source "DNSRecord.java"


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Ljavax/jmdns/impl/DNSRecord;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x9
    name = "IPv4Address"
.end annotation


# direct methods
.method constructor <init>(Ljava/lang/String;Ljavax/jmdns/impl/constants/DNSRecordClass;ZILjava/net/InetAddress;)V
    .registers 13
    .param p1, "name"    # Ljava/lang/String;
    .param p2, "recordClass"    # Ljavax/jmdns/impl/constants/DNSRecordClass;
    .param p3, "unique"    # Z
    .param p4, "ttl"    # I
    .param p5, "addr"    # Ljava/net/InetAddress;

    .line 176
    sget-object v2, Ljavax/jmdns/impl/constants/DNSRecordType;->TYPE_A:Ljavax/jmdns/impl/constants/DNSRecordType;

    move-object v0, p0

    move-object v1, p1

    move-object v3, p2

    move v4, p3

    move v5, p4

    move-object v6, p5

    invoke-direct/range {v0 .. v6}, Ljavax/jmdns/impl/DNSRecord$Address;-><init>(Ljava/lang/String;Ljavax/jmdns/impl/constants/DNSRecordType;Ljavax/jmdns/impl/constants/DNSRecordClass;ZILjava/net/InetAddress;)V

    .line 177
    return-void
.end method

.method constructor <init>(Ljava/lang/String;Ljavax/jmdns/impl/constants/DNSRecordClass;ZI[B)V
    .registers 13
    .param p1, "name"    # Ljava/lang/String;
    .param p2, "recordClass"    # Ljavax/jmdns/impl/constants/DNSRecordClass;
    .param p3, "unique"    # Z
    .param p4, "ttl"    # I
    .param p5, "rawAddress"    # [B

    .line 180
    sget-object v2, Ljavax/jmdns/impl/constants/DNSRecordType;->TYPE_A:Ljavax/jmdns/impl/constants/DNSRecordType;

    move-object v0, p0

    move-object v1, p1

    move-object v3, p2

    move v4, p3

    move v5, p4

    move-object v6, p5

    invoke-direct/range {v0 .. v6}, Ljavax/jmdns/impl/DNSRecord$Address;-><init>(Ljava/lang/String;Ljavax/jmdns/impl/constants/DNSRecordType;Ljavax/jmdns/impl/constants/DNSRecordClass;ZI[B)V

    .line 181
    return-void
.end method


# virtual methods
.method public getServiceInfo(Z)Ljavax/jmdns/ServiceInfo;
    .registers 4
    .param p1, "persistent"    # Z

    .line 208
    invoke-super {p0, p1}, Ljavax/jmdns/impl/DNSRecord$Address;->getServiceInfo(Z)Ljavax/jmdns/ServiceInfo;

    move-result-object v0

    check-cast v0, Ljavax/jmdns/impl/ServiceInfoImpl;

    .line 209
    .local v0, "info":Ljavax/jmdns/impl/ServiceInfoImpl;
    iget-object v1, p0, Ljavax/jmdns/impl/DNSRecord$IPv4Address;->_addr:Ljava/net/InetAddress;

    check-cast v1, Ljava/net/Inet4Address;

    invoke-virtual {v0, v1}, Ljavax/jmdns/impl/ServiceInfoImpl;->addAddress(Ljava/net/Inet4Address;)V

    .line 210
    return-object v0
.end method

.method write(Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;)V
    .registers 7
    .param p1, "out"    # Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;

    .line 185
    iget-object v0, p0, Ljavax/jmdns/impl/DNSRecord$IPv4Address;->_addr:Ljava/net/InetAddress;

    if-eqz v0, :cond_1f

    .line 186
    iget-object v0, p0, Ljavax/jmdns/impl/DNSRecord$IPv4Address;->_addr:Ljava/net/InetAddress;

    invoke-virtual {v0}, Ljava/net/InetAddress;->getAddress()[B

    move-result-object v0

    .line 188
    .local v0, "buffer":[B
    iget-object v1, p0, Ljavax/jmdns/impl/DNSRecord$IPv4Address;->_addr:Ljava/net/InetAddress;

    instance-of v1, v1, Ljava/net/Inet4Address;

    const/4 v2, 0x0

    if-eqz v1, :cond_12

    goto :goto_1b

    .line 192
    :cond_12
    move-object v1, v0

    .line 193
    .local v1, "tempbuffer":[B
    const/4 v3, 0x4

    new-array v0, v3, [B

    .line 194
    const/16 v4, 0xc

    invoke-static {v1, v4, v0, v2, v3}, Ljava/lang/System;->arraycopy(Ljava/lang/Object;ILjava/lang/Object;II)V

    .line 196
    .end local v1    # "tempbuffer":[B
    :goto_1b
    array-length v1, v0

    .line 197
    .local v1, "length":I
    invoke-virtual {p1, v0, v2, v1}, Ljavax/jmdns/impl/DNSOutgoing$MessageOutputStream;->writeBytes([BII)V

    .line 199
    .end local v0    # "buffer":[B
    .end local v1    # "length":I
    :cond_1f
    return-void
.end method
