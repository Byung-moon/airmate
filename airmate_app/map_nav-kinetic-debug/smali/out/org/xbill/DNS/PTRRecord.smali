.class public Lorg/xbill/DNS/PTRRecord;
.super Lorg/xbill/DNS/SingleCompressedNameBase;
.source "PTRRecord.java"


# static fields
.field private static final serialVersionUID:J = -0x737c6468424b3050L


# direct methods
.method constructor <init>()V
    .registers 1

    .line 16
    invoke-direct {p0}, Lorg/xbill/DNS/SingleCompressedNameBase;-><init>()V

    return-void
.end method

.method public constructor <init>(Lorg/xbill/DNS/Name;IJLorg/xbill/DNS/Name;)V
    .registers 14
    .param p1, "name"    # Lorg/xbill/DNS/Name;
    .param p2, "dclass"    # I
    .param p3, "ttl"    # J
    .param p5, "target"    # Lorg/xbill/DNS/Name;

    .line 29
    const-string v7, "target"

    const/16 v2, 0xc

    move-object v0, p0

    move-object v1, p1

    move v3, p2

    move-wide v4, p3

    move-object v6, p5

    invoke-direct/range {v0 .. v7}, Lorg/xbill/DNS/SingleCompressedNameBase;-><init>(Lorg/xbill/DNS/Name;IIJLorg/xbill/DNS/Name;Ljava/lang/String;)V

    .line 30
    return-void
.end method


# virtual methods
.method getObject()Lorg/xbill/DNS/Record;
    .registers 2

    .line 20
    new-instance v0, Lorg/xbill/DNS/PTRRecord;

    invoke-direct {v0}, Lorg/xbill/DNS/PTRRecord;-><init>()V

    return-object v0
.end method

.method public getTarget()Lorg/xbill/DNS/Name;
    .registers 2

    .line 35
    invoke-virtual {p0}, Lorg/xbill/DNS/PTRRecord;->getSingleName()Lorg/xbill/DNS/Name;

    move-result-object v0

    return-object v0
.end method
