.class final Lorg/apache/commons/io/FileCleaningTracker$Tracker;
.super Ljava/lang/ref/PhantomReference;
.source "FileCleaningTracker.java"


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Lorg/apache/commons/io/FileCleaningTracker;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x1a
    name = "Tracker"
.end annotation


# instance fields
.field private final deleteStrategy:Lorg/apache/commons/io/FileDeleteStrategy;

.field private final path:Ljava/lang/String;


# direct methods
.method constructor <init>(Ljava/lang/String;Lorg/apache/commons/io/FileDeleteStrategy;Ljava/lang/Object;Ljava/lang/ref/ReferenceQueue;)V
    .registers 6
    .param p1, "path"    # Ljava/lang/String;
    .param p2, "deleteStrategy"    # Lorg/apache/commons/io/FileDeleteStrategy;
    .param p3, "marker"    # Ljava/lang/Object;
    .param p4, "queue"    # Ljava/lang/ref/ReferenceQueue;

    .line 242
    invoke-direct {p0, p3, p4}, Ljava/lang/ref/PhantomReference;-><init>(Ljava/lang/Object;Ljava/lang/ref/ReferenceQueue;)V

    .line 243
    iput-object p1, p0, Lorg/apache/commons/io/FileCleaningTracker$Tracker;->path:Ljava/lang/String;

    .line 244
    if-nez p2, :cond_a

    sget-object v0, Lorg/apache/commons/io/FileDeleteStrategy;->NORMAL:Lorg/apache/commons/io/FileDeleteStrategy;

    goto :goto_b

    :cond_a
    move-object v0, p2

    :goto_b
    iput-object v0, p0, Lorg/apache/commons/io/FileCleaningTracker$Tracker;->deleteStrategy:Lorg/apache/commons/io/FileDeleteStrategy;

    .line 245
    return-void
.end method


# virtual methods
.method public delete()Z
    .registers 4

    .line 254
    iget-object v0, p0, Lorg/apache/commons/io/FileCleaningTracker$Tracker;->deleteStrategy:Lorg/apache/commons/io/FileDeleteStrategy;

    new-instance v1, Ljava/io/File;

    iget-object v2, p0, Lorg/apache/commons/io/FileCleaningTracker$Tracker;->path:Ljava/lang/String;

    invoke-direct {v1, v2}, Ljava/io/File;-><init>(Ljava/lang/String;)V

    invoke-virtual {v0, v1}, Lorg/apache/commons/io/FileDeleteStrategy;->deleteQuietly(Ljava/io/File;)Z

    move-result v0

    return v0
.end method
