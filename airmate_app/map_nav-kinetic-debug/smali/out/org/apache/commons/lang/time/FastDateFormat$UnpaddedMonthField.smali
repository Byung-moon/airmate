.class Lorg/apache/commons/lang/time/FastDateFormat$UnpaddedMonthField;
.super Ljava/lang/Object;
.source "FastDateFormat.java"

# interfaces
.implements Lorg/apache/commons/lang/time/FastDateFormat$NumberRule;


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Lorg/apache/commons/lang/time/FastDateFormat;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0xa
    name = "UnpaddedMonthField"
.end annotation


# static fields
.field static final INSTANCE:Lorg/apache/commons/lang/time/FastDateFormat$UnpaddedMonthField;


# direct methods
.method static constructor <clinit>()V
    .registers 1

    .line 1218
    new-instance v0, Lorg/apache/commons/lang/time/FastDateFormat$UnpaddedMonthField;

    invoke-direct {v0}, Lorg/apache/commons/lang/time/FastDateFormat$UnpaddedMonthField;-><init>()V

    sput-object v0, Lorg/apache/commons/lang/time/FastDateFormat$UnpaddedMonthField;->INSTANCE:Lorg/apache/commons/lang/time/FastDateFormat$UnpaddedMonthField;

    return-void
.end method

.method constructor <init>()V
    .registers 1

    .line 1225
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    .line 1226
    return-void
.end method


# virtual methods
.method public final appendTo(Ljava/lang/StringBuffer;I)V
    .registers 4
    .param p1, "buffer"    # Ljava/lang/StringBuffer;
    .param p2, "value"    # I

    .line 1246
    const/16 v0, 0xa

    if-ge p2, v0, :cond_b

    .line 1247
    add-int/lit8 v0, p2, 0x30

    int-to-char v0, v0

    invoke-virtual {p1, v0}, Ljava/lang/StringBuffer;->append(C)Ljava/lang/StringBuffer;

    goto :goto_1b

    .line 1249
    :cond_b
    div-int/lit8 v0, p2, 0xa

    add-int/lit8 v0, v0, 0x30

    int-to-char v0, v0

    invoke-virtual {p1, v0}, Ljava/lang/StringBuffer;->append(C)Ljava/lang/StringBuffer;

    .line 1250
    rem-int/lit8 v0, p2, 0xa

    add-int/lit8 v0, v0, 0x30

    int-to-char v0, v0

    invoke-virtual {p1, v0}, Ljava/lang/StringBuffer;->append(C)Ljava/lang/StringBuffer;

    .line 1252
    :goto_1b
    return-void
.end method

.method public appendTo(Ljava/lang/StringBuffer;Ljava/util/Calendar;)V
    .registers 4
    .param p1, "buffer"    # Ljava/lang/StringBuffer;
    .param p2, "calendar"    # Ljava/util/Calendar;

    .line 1239
    const/4 v0, 0x2

    invoke-virtual {p2, v0}, Ljava/util/Calendar;->get(I)I

    move-result v0

    add-int/lit8 v0, v0, 0x1

    invoke-virtual {p0, p1, v0}, Lorg/apache/commons/lang/time/FastDateFormat$UnpaddedMonthField;->appendTo(Ljava/lang/StringBuffer;I)V

    .line 1240
    return-void
.end method

.method public estimateLength()I
    .registers 2

    .line 1232
    const/4 v0, 0x2

    return v0
.end method
