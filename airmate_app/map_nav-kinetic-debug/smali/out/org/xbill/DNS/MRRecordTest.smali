.class public Lorg/xbill/DNS/MRRecordTest;
.super Ljunit/framework/TestCase;
.source "MRRecordTest.java"


# direct methods
.method public constructor <init>()V
    .registers 1

    .line 39
    invoke-direct {p0}, Ljunit/framework/TestCase;-><init>()V

    return-void
.end method


# virtual methods
.method public test_ctor_0arg()V
    .registers 3

    .line 43
    new-instance v0, Lorg/xbill/DNS/MRRecord;

    invoke-direct {v0}, Lorg/xbill/DNS/MRRecord;-><init>()V

    .line 44
    .local v0, "d":Lorg/xbill/DNS/MRRecord;
    invoke-virtual {v0}, Lorg/xbill/DNS/MRRecord;->getName()Lorg/xbill/DNS/Name;

    move-result-object v1

    invoke-static {v1}, Lorg/xbill/DNS/MRRecordTest;->assertNull(Ljava/lang/Object;)V

    .line 45
    invoke-virtual {v0}, Lorg/xbill/DNS/MRRecord;->getNewName()Lorg/xbill/DNS/Name;

    move-result-object v1

    invoke-static {v1}, Lorg/xbill/DNS/MRRecordTest;->assertNull(Ljava/lang/Object;)V

    .line 46
    return-void
.end method

.method public test_ctor_4arg()V
    .registers 10
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Lorg/xbill/DNS/TextParseException;
        }
    .end annotation

    .line 50
    const-string v0, "my.name."

    invoke-static {v0}, Lorg/xbill/DNS/Name;->fromString(Ljava/lang/String;)Lorg/xbill/DNS/Name;

    move-result-object v0

    .line 51
    .local v0, "n":Lorg/xbill/DNS/Name;
    const-string v1, "my.alias."

    invoke-static {v1}, Lorg/xbill/DNS/Name;->fromString(Ljava/lang/String;)Lorg/xbill/DNS/Name;

    move-result-object v7

    .line 53
    .local v7, "a":Lorg/xbill/DNS/Name;
    new-instance v8, Lorg/xbill/DNS/MRRecord;

    const/4 v3, 0x1

    const-wide/32 v4, 0xabcde

    move-object v1, v8

    move-object v2, v0

    move-object v6, v7

    invoke-direct/range {v1 .. v6}, Lorg/xbill/DNS/MRRecord;-><init>(Lorg/xbill/DNS/Name;IJLorg/xbill/DNS/Name;)V

    .line 54
    .local v1, "d":Lorg/xbill/DNS/MRRecord;
    invoke-virtual {v1}, Lorg/xbill/DNS/MRRecord;->getName()Lorg/xbill/DNS/Name;

    move-result-object v2

    invoke-static {v0, v2}, Lorg/xbill/DNS/MRRecordTest;->assertEquals(Ljava/lang/Object;Ljava/lang/Object;)V

    .line 55
    invoke-virtual {v1}, Lorg/xbill/DNS/MRRecord;->getType()I

    move-result v2

    const/16 v3, 0x9

    invoke-static {v3, v2}, Lorg/xbill/DNS/MRRecordTest;->assertEquals(II)V

    .line 56
    invoke-virtual {v1}, Lorg/xbill/DNS/MRRecord;->getDClass()I

    move-result v2

    const/4 v3, 0x1

    invoke-static {v3, v2}, Lorg/xbill/DNS/MRRecordTest;->assertEquals(II)V

    .line 57
    invoke-virtual {v1}, Lorg/xbill/DNS/MRRecord;->getTTL()J

    move-result-wide v2

    invoke-static {v4, v5, v2, v3}, Lorg/xbill/DNS/MRRecordTest;->assertEquals(JJ)V

    .line 58
    invoke-virtual {v1}, Lorg/xbill/DNS/MRRecord;->getNewName()Lorg/xbill/DNS/Name;

    move-result-object v2

    invoke-static {v7, v2}, Lorg/xbill/DNS/MRRecordTest;->assertEquals(Ljava/lang/Object;Ljava/lang/Object;)V

    .line 59
    return-void
.end method

.method public test_getObject()V
    .registers 4

    .line 63
    new-instance v0, Lorg/xbill/DNS/MRRecord;

    invoke-direct {v0}, Lorg/xbill/DNS/MRRecord;-><init>()V

    .line 64
    .local v0, "d":Lorg/xbill/DNS/MRRecord;
    invoke-virtual {v0}, Lorg/xbill/DNS/MRRecord;->getObject()Lorg/xbill/DNS/Record;

    move-result-object v1

    .line 65
    .local v1, "r":Lorg/xbill/DNS/Record;
    instance-of v2, v1, Lorg/xbill/DNS/MRRecord;

    invoke-static {v2}, Lorg/xbill/DNS/MRRecordTest;->assertTrue(Z)V

    .line 66
    return-void
.end method
