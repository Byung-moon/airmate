.class public Lorg/apache/commons/io/filefilter/EmptyFileFilter;
.super Lorg/apache/commons/io/filefilter/AbstractFileFilter;
.source "EmptyFileFilter.java"

# interfaces
.implements Ljava/io/Serializable;


# static fields
.field public static final EMPTY:Lorg/apache/commons/io/filefilter/IOFileFilter;

.field public static final NOT_EMPTY:Lorg/apache/commons/io/filefilter/IOFileFilter;


# direct methods
.method static constructor <clinit>()V
    .registers 2

    .line 57
    new-instance v0, Lorg/apache/commons/io/filefilter/EmptyFileFilter;

    invoke-direct {v0}, Lorg/apache/commons/io/filefilter/EmptyFileFilter;-><init>()V

    sput-object v0, Lorg/apache/commons/io/filefilter/EmptyFileFilter;->EMPTY:Lorg/apache/commons/io/filefilter/IOFileFilter;

    .line 60
    new-instance v0, Lorg/apache/commons/io/filefilter/NotFileFilter;

    sget-object v1, Lorg/apache/commons/io/filefilter/EmptyFileFilter;->EMPTY:Lorg/apache/commons/io/filefilter/IOFileFilter;

    invoke-direct {v0, v1}, Lorg/apache/commons/io/filefilter/NotFileFilter;-><init>(Lorg/apache/commons/io/filefilter/IOFileFilter;)V

    sput-object v0, Lorg/apache/commons/io/filefilter/EmptyFileFilter;->NOT_EMPTY:Lorg/apache/commons/io/filefilter/IOFileFilter;

    return-void
.end method

.method protected constructor <init>()V
    .registers 1

    .line 65
    invoke-direct {p0}, Lorg/apache/commons/io/filefilter/AbstractFileFilter;-><init>()V

    .line 66
    return-void
.end method


# virtual methods
.method public accept(Ljava/io/File;)Z
    .registers 9
    .param p1, "file"    # Ljava/io/File;

    .line 76
    invoke-virtual {p1}, Ljava/io/File;->isDirectory()Z

    move-result v0

    const/4 v1, 0x0

    const/4 v2, 0x1

    if-eqz v0, :cond_15

    .line 77
    invoke-virtual {p1}, Ljava/io/File;->listFiles()[Ljava/io/File;

    move-result-object v0

    .line 78
    .local v0, "files":[Ljava/io/File;
    if-eqz v0, :cond_13

    array-length v3, v0

    if-nez v3, :cond_12

    goto :goto_13

    :cond_12
    goto :goto_14

    :cond_13
    :goto_13
    const/4 v1, 0x1

    :goto_14
    return v1

    .line 80
    .end local v0    # "files":[Ljava/io/File;
    :cond_15
    invoke-virtual {p1}, Ljava/io/File;->length()J

    move-result-wide v3

    const-wide/16 v5, 0x0

    cmp-long v0, v3, v5

    if-nez v0, :cond_21

    const/4 v1, 0x1

    nop

    :cond_21
    return v1
.end method