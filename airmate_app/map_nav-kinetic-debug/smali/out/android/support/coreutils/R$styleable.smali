.class public final Landroid/support/coreutils/R$styleable;
.super Ljava/lang/Object;
.source "R.java"


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Landroid/support/coreutils/R;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x19
    name = "styleable"
.end annotation


# static fields
.field public static final FontFamily:[I

.field public static final FontFamilyFont:[I

.field public static final FontFamilyFont_android_font:I = 0x0

.field public static final FontFamilyFont_android_fontStyle:I = 0x2

.field public static final FontFamilyFont_android_fontWeight:I = 0x1

.field public static final FontFamilyFont_font:I = 0x3

.field public static final FontFamilyFont_fontStyle:I = 0x4

.field public static final FontFamilyFont_fontWeight:I = 0x5

.field public static final FontFamily_fontProviderAuthority:I = 0x0

.field public static final FontFamily_fontProviderCerts:I = 0x1

.field public static final FontFamily_fontProviderFetchStrategy:I = 0x2

.field public static final FontFamily_fontProviderFetchTimeout:I = 0x3

.field public static final FontFamily_fontProviderPackage:I = 0x4

.field public static final FontFamily_fontProviderQuery:I = 0x5


# direct methods
.method static constructor <clinit>()V
    .registers 2

    .line 142
    const/4 v0, 0x6

    new-array v1, v0, [I

    fill-array-data v1, :array_10

    sput-object v1, Landroid/support/coreutils/R$styleable;->FontFamily:[I

    .line 149
    new-array v0, v0, [I

    fill-array-data v0, :array_20

    sput-object v0, Landroid/support/coreutils/R$styleable;->FontFamilyFont:[I

    return-void

    :array_10
    .array-data 4
        0x7f020073
        0x7f020074
        0x7f020075
        0x7f020076
        0x7f020077
        0x7f020078
    .end array-data

    :array_20
    .array-data 4
        0x1010532
        0x1010533
        0x101053f
        0x7f020071
        0x7f020079
        0x7f02007a
    .end array-data
.end method

.method private constructor <init>()V
    .registers 1

    .line 140
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    return-void
.end method
