.class Landroid/support/v4/provider/SingleDocumentFile;
.super Landroid/support/v4/provider/DocumentFile;
.source "SingleDocumentFile.java"


# annotations
.annotation build Landroid/support/annotation/RequiresApi;
    value = 0x13
.end annotation


# instance fields
.field private mContext:Landroid/content/Context;

.field private mUri:Landroid/net/Uri;


# direct methods
.method constructor <init>(Landroid/support/v4/provider/DocumentFile;Landroid/content/Context;Landroid/net/Uri;)V
    .registers 4
    .param p1, "parent"    # Landroid/support/v4/provider/DocumentFile;
    .param p2, "context"    # Landroid/content/Context;
    .param p3, "uri"    # Landroid/net/Uri;

    .line 30
    invoke-direct {p0, p1}, Landroid/support/v4/provider/DocumentFile;-><init>(Landroid/support/v4/provider/DocumentFile;)V

    .line 31
    iput-object p2, p0, Landroid/support/v4/provider/SingleDocumentFile;->mContext:Landroid/content/Context;

    .line 32
    iput-object p3, p0, Landroid/support/v4/provider/SingleDocumentFile;->mUri:Landroid/net/Uri;

    .line 33
    return-void
.end method


# virtual methods
.method public canRead()Z
    .registers 3

    .line 87
    iget-object v0, p0, Landroid/support/v4/provider/SingleDocumentFile;->mContext:Landroid/content/Context;

    iget-object v1, p0, Landroid/support/v4/provider/SingleDocumentFile;->mUri:Landroid/net/Uri;

    invoke-static {v0, v1}, Landroid/support/v4/provider/DocumentsContractApi19;->canRead(Landroid/content/Context;Landroid/net/Uri;)Z

    move-result v0

    return v0
.end method

.method public canWrite()Z
    .registers 3

    .line 92
    iget-object v0, p0, Landroid/support/v4/provider/SingleDocumentFile;->mContext:Landroid/content/Context;

    iget-object v1, p0, Landroid/support/v4/provider/SingleDocumentFile;->mUri:Landroid/net/Uri;

    invoke-static {v0, v1}, Landroid/support/v4/provider/DocumentsContractApi19;->canWrite(Landroid/content/Context;Landroid/net/Uri;)Z

    move-result v0

    return v0
.end method

.method public createDirectory(Ljava/lang/String;)Landroid/support/v4/provider/DocumentFile;
    .registers 3
    .param p1, "displayName"    # Ljava/lang/String;

    .line 42
    new-instance v0, Ljava/lang/UnsupportedOperationException;

    invoke-direct {v0}, Ljava/lang/UnsupportedOperationException;-><init>()V

    throw v0
.end method

.method public createFile(Ljava/lang/String;Ljava/lang/String;)Landroid/support/v4/provider/DocumentFile;
    .registers 4
    .param p1, "mimeType"    # Ljava/lang/String;
    .param p2, "displayName"    # Ljava/lang/String;

    .line 37
    new-instance v0, Ljava/lang/UnsupportedOperationException;

    invoke-direct {v0}, Ljava/lang/UnsupportedOperationException;-><init>()V

    throw v0
.end method

.method public delete()Z
    .registers 3

    .line 98
    :try_start_0
    iget-object v0, p0, Landroid/support/v4/provider/SingleDocumentFile;->mContext:Landroid/content/Context;

    invoke-virtual {v0}, Landroid/content/Context;->getContentResolver()Landroid/content/ContentResolver;

    move-result-object v0

    iget-object v1, p0, Landroid/support/v4/provider/SingleDocumentFile;->mUri:Landroid/net/Uri;

    invoke-static {v0, v1}, Landroid/provider/DocumentsContract;->deleteDocument(Landroid/content/ContentResolver;Landroid/net/Uri;)Z

    move-result v0
    :try_end_c
    .catch Ljava/lang/Exception; {:try_start_0 .. :try_end_c} :catch_d

    return v0

    .line 99
    :catch_d
    move-exception v0

    .line 100
    .local v0, "e":Ljava/lang/Exception;
    const/4 v1, 0x0

    return v1
.end method

.method public exists()Z
    .registers 3

    .line 106
    iget-object v0, p0, Landroid/support/v4/provider/SingleDocumentFile;->mContext:Landroid/content/Context;

    iget-object v1, p0, Landroid/support/v4/provider/SingleDocumentFile;->mUri:Landroid/net/Uri;

    invoke-static {v0, v1}, Landroid/support/v4/provider/DocumentsContractApi19;->exists(Landroid/content/Context;Landroid/net/Uri;)Z

    move-result v0

    return v0
.end method

.method public getName()Ljava/lang/String;
    .registers 3

    .line 52
    iget-object v0, p0, Landroid/support/v4/provider/SingleDocumentFile;->mContext:Landroid/content/Context;

    iget-object v1, p0, Landroid/support/v4/provider/SingleDocumentFile;->mUri:Landroid/net/Uri;

    invoke-static {v0, v1}, Landroid/support/v4/provider/DocumentsContractApi19;->getName(Landroid/content/Context;Landroid/net/Uri;)Ljava/lang/String;

    move-result-object v0

    return-object v0
.end method

.method public getType()Ljava/lang/String;
    .registers 3

    .line 57
    iget-object v0, p0, Landroid/support/v4/provider/SingleDocumentFile;->mContext:Landroid/content/Context;

    iget-object v1, p0, Landroid/support/v4/provider/SingleDocumentFile;->mUri:Landroid/net/Uri;

    invoke-static {v0, v1}, Landroid/support/v4/provider/DocumentsContractApi19;->getType(Landroid/content/Context;Landroid/net/Uri;)Ljava/lang/String;

    move-result-object v0

    return-object v0
.end method

.method public getUri()Landroid/net/Uri;
    .registers 2

    .line 47
    iget-object v0, p0, Landroid/support/v4/provider/SingleDocumentFile;->mUri:Landroid/net/Uri;

    return-object v0
.end method

.method public isDirectory()Z
    .registers 3

    .line 62
    iget-object v0, p0, Landroid/support/v4/provider/SingleDocumentFile;->mContext:Landroid/content/Context;

    iget-object v1, p0, Landroid/support/v4/provider/SingleDocumentFile;->mUri:Landroid/net/Uri;

    invoke-static {v0, v1}, Landroid/support/v4/provider/DocumentsContractApi19;->isDirectory(Landroid/content/Context;Landroid/net/Uri;)Z

    move-result v0

    return v0
.end method

.method public isFile()Z
    .registers 3

    .line 67
    iget-object v0, p0, Landroid/support/v4/provider/SingleDocumentFile;->mContext:Landroid/content/Context;

    iget-object v1, p0, Landroid/support/v4/provider/SingleDocumentFile;->mUri:Landroid/net/Uri;

    invoke-static {v0, v1}, Landroid/support/v4/provider/DocumentsContractApi19;->isFile(Landroid/content/Context;Landroid/net/Uri;)Z

    move-result v0

    return v0
.end method

.method public isVirtual()Z
    .registers 3

    .line 72
    iget-object v0, p0, Landroid/support/v4/provider/SingleDocumentFile;->mContext:Landroid/content/Context;

    iget-object v1, p0, Landroid/support/v4/provider/SingleDocumentFile;->mUri:Landroid/net/Uri;

    invoke-static {v0, v1}, Landroid/support/v4/provider/DocumentsContractApi19;->isVirtual(Landroid/content/Context;Landroid/net/Uri;)Z

    move-result v0

    return v0
.end method

.method public lastModified()J
    .registers 3

    .line 77
    iget-object v0, p0, Landroid/support/v4/provider/SingleDocumentFile;->mContext:Landroid/content/Context;

    iget-object v1, p0, Landroid/support/v4/provider/SingleDocumentFile;->mUri:Landroid/net/Uri;

    invoke-static {v0, v1}, Landroid/support/v4/provider/DocumentsContractApi19;->lastModified(Landroid/content/Context;Landroid/net/Uri;)J

    move-result-wide v0

    return-wide v0
.end method

.method public length()J
    .registers 3

    .line 82
    iget-object v0, p0, Landroid/support/v4/provider/SingleDocumentFile;->mContext:Landroid/content/Context;

    iget-object v1, p0, Landroid/support/v4/provider/SingleDocumentFile;->mUri:Landroid/net/Uri;

    invoke-static {v0, v1}, Landroid/support/v4/provider/DocumentsContractApi19;->length(Landroid/content/Context;Landroid/net/Uri;)J

    move-result-wide v0

    return-wide v0
.end method

.method public listFiles()[Landroid/support/v4/provider/DocumentFile;
    .registers 2

    .line 111
    new-instance v0, Ljava/lang/UnsupportedOperationException;

    invoke-direct {v0}, Ljava/lang/UnsupportedOperationException;-><init>()V

    throw v0
.end method

.method public renameTo(Ljava/lang/String;)Z
    .registers 3
    .param p1, "displayName"    # Ljava/lang/String;

    .line 116
    new-instance v0, Ljava/lang/UnsupportedOperationException;

    invoke-direct {v0}, Ljava/lang/UnsupportedOperationException;-><init>()V

    throw v0
.end method
