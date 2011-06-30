#define MSG_TIMEOUT "Onderbreking"
#define MSG_CAPABILITIES "Onmogelijk om de instelling te wisselen"
#define MSG_CLIENT_CLOSED_CNX "Client sloot de verbinding"
#define MSG_CLIENT_READ_ERR "Leesfout van de client"
#define MSG_CANT_OPEN_CNX "Kan geen verbinding openen"
#define MSG_CANT_CREATE_DATA_SOCKET "Kan geen data socket aanmaken"
#define MSG_DEBUG_CLIENT_IS "Het client adres is: "
#define MSG_SYNTAX_ERROR_IP "Syntax fout in IP adres"
#define MSG_PORT_SUCCESSFUL "PORT commando geslaagd"
#define MSG_ONLY_IPV4V6 "Alleen IPv4 en IPv6 worden ondersteund (1,2)"
#define MSG_ONLY_IPV4 "Alleen IPv4 wordt ondersteund (1)"
#define MSG_TIMEOUT_PARSER "Onderbreking - volgende keer wat sneller typen a.u.b."
#define MSG_LINE_TOO_LONG "Regel is te lang"
#define MSG_LOG_OVERFLOW "De client probeerde teveel in te geven in de commandoregel buffer"
#define MSG_GOODBYE "Tot ziens. U heeft %llu kbytes ge-upload en %llu kbytes ge-download"
#define MSG_DEBUG_COMMAND "Commando"
#define MSG_IS_YOUR_CURRENT_LOCATION "is uw huidige locatie"
#define MSG_NOT_LOGGED_IN "U bent niet ingelogd"
#define MSG_AUTH_UNIMPLEMENTED "Beveiligings uitbreidingen zijn niet opgenomen"
#define MSG_NO_FILE_NAME "Geen bestands naam"
#define MSG_NO_DIRECTORY_NAME "Geen directory naam"
#define MSG_NO_RESTART_POINT "Geen nieuw beginpunt"
#define MSG_ABOR_SUCCESS "Als u dit ziet is ABOR geslaagd"
#define MSG_MISSING_ARG "Argument ontbreekt"
#define MSG_GARBAGE_FOUND "Onbruikbare gegevens gevonden na ingegeven waarde"
#define MSG_VALUE_TOO_LARGE "Waarde te groot"
#define MSG_IDLE_TIME "Inactieve tijd ingesteld op %lu seconden"
#define MSG_SITE_HELP "De volgende SITE commando's werden herkend"
#define MSG_BAD_CHMOD "Ongeldige permissies"
#define MSG_UNKNOWN_EXTENSION "is een onbekende extensie"
#define MSG_XDBG_OK "XDBG commando is geslaagd, debug level is nu %d"
#define MSG_UNKNOWN_COMMAND "Onbekend commando"
#define MSG_TIMEOUT_NOOP "Onderbreking (geen activiteit gedurende %lu seconden)"
#define MSG_TIMEOUT_DATA "Onderbreking (geen nieuwe gegevens gedurende %lu seconden)"
#define MSG_SLEEPING "Zzz..."
#define MSG_ALREADY_LOGGED "U bent reeds ingelogd!"
#define MSG_ANY_PASSWORD "Elk willekeurig wachtwoord is goed"
#define MSG_ANONYMOUS_LOGGED "Anonieme gebruiker is ingelogd"
#define MSG_ANONYMOUS_LOGGED_VIRTUAL "Anonieme gebruiker is ingelogd in de virtuele FTP"
#define MSG_USER_OK "Gebruiker %s OK. Wachtwoord vereist"
#define MSG_CANT_DO_TWICE "In de huidige sessie kunnen we dit niet doen"
#define MSG_UNABLE_SECURE_ANON "Het is onmogelijk om een beveiligde anonieme FTP op te zetten"
#define MSG_BANDWIDTH_RESTRICTED "Uw bandbreedte is beperkt"
#define MSG_NO_PASSWORD_NEEDED "Ik heb geen wachtwoord nodig!"
#define MSG_NOTRUST "Het spijt me, maar ik kan u niet vertrouwen"
#define MSG_WHOAREYOU "Wie bent U?"
#define MSG_AUTH_FAILED "Sorry, autorisatie is mislukt"
#define MSG_AUTH_TOOMANY "Autorisatie is te vaak mislukt"
#define MSG_NO_HOMEDIR "Home directory niet beschikbaar - sessie wordt afgebroken"
#define MSG_NO_HOMEDIR2 "%s bestaat niet of is onbereikbaar"
#define MSG_START_SLASH "U begint in /"
#define MSG_USER_GROUP_ACCESS "Gebruiker %s heeft groepstoegang voor"
#define MSG_FXP_SUPPORT "Deze server ondersteunt FXP overdracht"
#define MSG_RATIO "U moet voldoen aan een %u:%u (UL/DL) verhouding"
#define MSG_CHROOT_FAILED "Het is niet mogelijk een veilige chroot() op te zetten"
#define MSG_CURRENT_DIR_IS "OK. De huidige directory is %s"
#define MSG_CURRENT_RESTRICTED_DIR_IS "OK. De huidige beperkte directory is %s"
#define MSG_IS_NOW_LOGGED_IN "%s is nu ingelogd"
#define MSG_CANT_CHANGE_DIR "Kan de directory niet veranderen naar %s"
#define MSG_PATH_TOO_LONG "Pad is te lang"
#define MSG_CANT_PASV "U kan PASV niet gebruiken bij IPv6 verbindingen. Gebruik EPSV."
#define MSG_CANT_PASSIVE "Het is onmogelijk een passieve verbinding te openen"
#define MSG_PORTS_BUSY "Alle gereserveerde TCP poorten zijn in gebruik"
#define MSG_GETSOCKNAME_DATA "Het is onmogelijk de lokale data socket te identificeren"
#define MSG_GETPEERNAME "Het is onmogelijk de lokale socket te identificeren "
#define MSG_INVALID_IP "Sorry, ongeldig adres opgegeven"
#define MSG_NO_EPSV "Een IPv6-conformant client met EPSV ondersteuning gebruiken a.u.b."
#define MSG_BAD_PORT "Sorry, maar ik gebruik geen poorten < 1024"
#define MSG_NO_FXP "Ik ben niet van plan een verbinding te openen met %s (alleen met %s)"
#define MSG_FXP "FXP overdracht: van %s naar %s"
#define MSG_NO_DATA_CONN "Geen data verbinding"
#define MSG_ACCEPT_FAILED "De verbinding kon niet gemaakt worden"
#define MSG_ACCEPT_SUCCESS "Data verbinding gemaakt"
#define MSG_CNX_PORT_FAILED "Kon geen data verbinding openen naar poort %d"
#define MSG_CNX_PORT "Bezig te verbinden met poort %d"
#define MSG_ANON_CANT_MKD "Sorry, anonieme gebruikers kunnen geen directories aanmaken"
#define MSG_ANON_CANT_RMD "Sorry, anonieme gebruikers kunnen geen directories verwijderen"
#define MSG_ANON_CANT_RENAME "Anonieme gebruikers kunnen geen bestanden verplaatsen/herbenoemen"
#define MSG_ANON_CANT_CHANGE_PERMS "Anonieme gebruikers kunnen geen permissies veranderen"
#define MSG_GLOB_NO_MEMORY "Niet genoeg geheugen tijdens globbing van %s"
#define MSG_PROBABLY_DENIED "(Dit betekent schijnbaar \"Hiervoor is toestemming nodig\")"
#define MSG_GLOB_READ_ERROR "Leesfout tijdens globbing van %s"
#define MSG_GLOB_NO_MATCH "Geen overeenstemming voor %s in %s"
#define MSG_CHMOD_FAILED "Kon de permissies op %s niet veranderen"
#define MSG_CHMOD_SUCCESS "Permissies veranderd op %s"
#define MSG_CHMOD_TOTAL_FAILURE "Sorry, maar ik kon geen enkele permissie veranderen"
#define MSG_ANON_CANT_DELETE "Anonieme gebruikers kunnen geen bestanden wissen"
#define MSG_ANON_CANT_OVERWRITE "Anonieme gebruikers kunnen geen bestaande bestanden overschrijven."
#define MSG_DELE_FAILED "Kon %s niet wissen"
#define MSG_DELE_SUCCESS "%s%s%s%s gewist"
#define MSG_DELE_TOTAL_FAILURE "Geen bestand gewist"
#define MSG_LOAD_TOO_HIGH \
    "De belasting was %3.2f toen u verbinding kreeg. Wij staan geen\n" \
        "downloads toe door anonieme gebruikers bij zo'n hoge belasting.\n" \
        "Uploaden is altijd toegestaan."
#define MSG_OPEN_FAILURE "Kan %s niet openen"
#define MSG_OPEN_FAILURE2 "Kan dat bestand niet openen"
#define MSG_STAT_FAILURE "Kan die bestandsgrootte niet vinden"
#define MSG_STAT_FAILURE2 "Kan niet kijken of dat bestand bestaat"
#define MSG_REST_TOO_LARGE_FOR_FILE "Opnieuw opgestart offset %lld is te groot voor bestand grootte %lld."
#define MSG_REST_RESET "Opnieuw opgestart, offset is weer op 0 gezet"
#define MSG_NOT_REGULAR_FILE "Ik kan alleen standaard bestanden herstellen"
#define MSG_NOT_MODERATED \
    "Dit bestand is door een anonieme gebruiker ge-upload. Er is door de \n" \
        "site beheerder nog geen toestemming gegegeven om die te downloaden.\n"
#define MSG_RATIO_DENIAL \
    "Het spijt me, maar de upload/download verhouding is %u:%u .\n" \
    "Tot nu toe heeft u %llu Kb ge-upload en %llu Kb ge-download \n" \
    "Upload eens wat leuks en probeer het later nog eens."
#define MSG_NO_MORE_TO_DOWNLOAD "Er is niets meer om te downloaden"
#define MSG_WINNER "De computer is uw vriend. Vertrouw maar op de computer"
#define MSG_KBYTES_LEFT "%.1f kbytes om te downloaden"
#define MSG_ABORTED "Overdracht afgebroken"
#define MSG_DATA_WRITE_FAILED "Fout tijdens schrijven naar data verbinding"
#define MSG_DATA_READ_FAILED "Fout tijdens lezen van data verbinding "
#define MSG_MMAP_FAILED "Onmogelijk het bestand in het geheugen te plaatsen"
#define MSG_WRITE_FAILED "Fout tijdens schrijven naar bestand"
#define MSG_TRANSFER_RATE_M "%.3f seconden (hier gemeten), %.2f Mbytes per seconde"
#define MSG_TRANSFER_RATE_K "%.3f seconden (hier gemeten), %.2f Kbytes per seconde"
#define MSG_TRANSFER_RATE_B "%.3f seconden (hier gemeten), %.2f bytes per seconde"
#define MSG_SPACE_FREE_M "%.1f Mbytes vrije schijfruimte"
#define MSG_SPACE_FREE_K "%f Kbytes vrije schijfruimte "
#define MSG_DOWNLOADED "gedownload"
#define MSG_REST_NOT_NUMERIC "REST verlangt een numerieke parameter"
#define MSG_REST_ASCII_STRICT "Antwoord teken moet 0 zijn in ASCII mode"
#define MSG_REST_ASCII_WORKAROUND "We beginnen opnieuw bij %lld. Maar we zijn in ASCII mode"
#define MSG_REST_SUCCESS "We beginnen opnieuw bij %lld"
#define MSG_SANITY_DIRECTORY_FAILURE "Verboden directory naam"
#define MSG_SANITY_FILE_FAILURE "Verboden bestandsnaam: %s"
#define MSG_MKD_FAILURE "Kan geen directory aanmaken"
#define MSG_MKD_SUCCESS "De directory is met succes aangemaakt"
#define MSG_RMD_FAILURE "Kan de directory niet verwijderen"
#define MSG_RMD_SUCCESS "De directory is verwijderd"
#define MSG_TIMESTAMP_FAILURE "Kan geen tijdsindicatie krijgen"
#define MSG_MODE_ERROR "Alleen ASCII en binary modes worden ondersteund"
#define MSG_CREATE_FAILURE "Niet mogelijk om een bestand aan te maken"
#define MSG_ABRT_ONLY "ABRT is het enige geldige commando tijdens uploaden"
#define MSG_UPLOAD_PARTIAL "gedeeltelijk ge-upload"
#define MSG_REMOVED "verwijderd"
#define MSG_UPLOADED "ge-upload"
#define MSG_GMTIME_FAILURE "Kon de lokale tijd niet uitlezen"
#define MSG_TYPE_8BIT_FAILURE "Alleen 8-bit bytes worden ondersteund, we leven niet in de vorige eeuw"
#define MSG_TYPE_UNKNOWN "Onbekend TYPE"
#define MSG_TYPE_SUCCESS "TYPE is nu"
#define MSG_STRU_FAILURE "Alleen F(ile) wordt ondersteund"
#define MSG_MODE_FAILURE "S(tream) mode gebruiken a.u.b."
#define MSG_RENAME_ABORT "Voorgaande naamsverandering wordt afgebroken"
#define MSG_RENAME_RNFR_SUCCESS "RNFR geaccepteerd - bestand bestaat reeds, gereed voor bestemming"
#define MSG_FILE_DOESNT_EXIST "Het spijt me, maar dat bestand bestaat niet"
#define MSG_RENAME_ALREADY_THERE "RENAME mislukt - doelbestand  bestaat reeds"
#define MSG_RENAME_NORNFR "RNFR is nodig, v��r RNTO"
#define MSG_RENAME_FAILURE "Herbenoemings/verplaatsings fout"
#define MSG_RENAME_SUCCESS "Bestand is met succes hernoemd of verplaatst"
#define MSG_NO_SUPERSERVER "Pure-ftpd binnen een super-server (zoals tcpserver) laten draaien"
#define MSG_NO_FTP_ACCOUNT "Niet in staat om de 'ftp' account te vinden"
#define MSG_CONF_ERR "Configuratie fout"
#define MSG_NO_VIRTUAL_FILE "Virtual users bestandsnaam ontbreekt"
#define MSG_ILLEGAL_THROTTLING "Niet toegestane waarde voor throttling"
#define MSG_ILLEGAL_TRUSTED_GID "Niet toegestane trusted gid voor chroot"
#define MSG_ILLEGAL_USER_LIMIT "Niet toegestane gebruikers-limiet"
#define MSG_ILLEGAL_FACILITY "Onbekende facility naam"
#define MSG_ILLEGAL_CONFIG_FILE_LDAP "Ongeldige LDAP configuratiebestand"
#define MSG_ILLEGAL_LOAD_LIMIT "Niet toegestane load limiet"
#define MSG_ILLEGAL_PORTS_RANGE "Niet toegestaan poorten bereik"
#define MSG_ILLEGAL_LS_LIMITS "Niet toegestane 'ls' limieten"
#define MSG_ILLEGAL_FORCE_PASSIVE "Niet toegestaan gedwongen IP voor passieve verbindingen"
#define MSG_ILLEGAL_RATIO "Niet toegestane upload/download verhouding"
#define MSG_ILLEGAL_UID_LIMIT "Niet toegestane uid limiet"
#define MSG_ILLEGAL_OPTION "Onbekende run-time optie"
#define MSG_LDAP_MISSING_BASE "Ontbrekende LDAPBaseDN in de LDAP configuratiebestand "
#define MSG_LDAP_WRONG_PARMS "Verkeerde LDAP parameters"
#define MSG_NEW_CONNECTION "Nieuwe verbinding vanaf %s"
#define MSG_WELCOME_TO "Welkom bij"
#define MSG_MAX_USERS "%lu gebruikers (het maximum) zijn reeds ingelogd, sorry"
#define MSG_NB_USERS "U bent gebruiker nummer %d van %d toegestane gebruikers"
#define MSG_WELCOME_TIME "Locale tijd is nu %02d:%02d. Server poort: %u."
#define MSG_ANONYMOUS_FTP_ONLY "Alleen anonieme FTP is hier toegestaan"
#define MSG_RATIOS_EVERYONE "UL/DL VERHOUDINGEN ZIJN VOOR IEDEREEN INGESCHAKELD:"
#define MSG_RATIOS_ANONYMOUS "ANONIEME GEBRUIKERS KRIJGEN EEN UL/DL VERHOUDING :"
#define MSG_RATIOS_RULE "%u Mb te downloaden, uploaden van %u Mb is verplicht."
#define MSG_INFO_IDLE_M "De verbinding wordt na %lu minuten van inactiviteit verbroken."
#define MSG_INFO_IDLE_S "De verbinding wordt na %lu seconden van inactiviteit verbroken."
#define MSG_CANT_READ_FILE "Sorry, we konden dit niet lezen [%s]"
#define MSG_LS_TRUNCATED "Output afgekort naar %u overeenstemmingen"
#define MSG_LS_SUCCESS "in totaal %u overeenstemmingen"
#define MSG_LOGOUT "Logout."
#define MSG_AUTH_FAILED_LOG "Autorisatie faalde voor gebruiker [%s]"
#define MSG_ILLEGAL_UMASK "Ongeldige umask"
#define MSG_STANDALONE_FAILED "Niet mogelijk om een standalone server te starten"
#define MSG_NO_ANONYMOUS_LOGIN "Dit is een priv� systeem - Geen anonieme login mogelijk"
#define MSG_ANONYMOUS_ANY_PASSWORD "Elk willekeurig wachtwoord is goed"
#define MSG_MAX_USERS_IP "Teveel verbindingen (%lu) vanaf dit IP"
#define MSG_ACTIVE_DISABLED "Active mode is uitgeschakeld"
#define MSG_TRANSFER_SUCCESSFUL "Bestand overdracht was succesvol"
#define MSG_NO_DISK_SPACE "Schijf is vol - later uploaden a.u.b."
#define MSG_OUT_OF_MEMORY "Geen geheugen meer!"
#define MSG_ILLEGAL_TRUSTED_IP "Ilegaal IP adres"
#define MSG_NO_ASCII_RESUME "ASCII hervatten is onveilig, verwijder het bestand eerst"
#define MSG_UNKNOWN_ALTLOG "Onbekend log formaat"
#define MSG_ACCOUNT_DISABLED "U kan niet inloggen als [%s] : het account is uitgeschakeld"
#define MSG_SQL_WRONG_PARMS "Verkeerde SQL parameters"
#define MSG_ILLEGAL_CONFIG_FILE_SQL "Ongeldig SQL configuratie bestand"
#define MSG_SQL_MISSING_SERVER "De server ontbreekt in het SQL configuratie bestand"
#define MSG_SQL_DOWN "De SQL server lijkt niet actief te zijn"
#define MSG_ILLEGAL_QUOTA "Ongeldige quota"
#define MSG_QUOTA_FILES "%llu Bestanden in gebruik (%d%%) - goedgekeurd: %llu bestanden"
#define MSG_QUOTA_SIZE "%llu Kbytes in gebruik (%d%%) - goedgekeurd: %llu Kb"
#define MSG_QUOTA_EXCEEDED "Quota overschreden: [%s] worden niet opgeslagen"
#define MSG_AUTH_UNKNOWN "Onbekende autorisatie methode"
#define MSG_PDB_BROKEN "Kan het geindexeerde  puredb bestand niet lezen (of oud formaat gedetecteerd) - Probeer pure-pw mkdb"
#define MSG_ALIASES_ALIAS "%s is een alias voor %s."
#define MSG_ALIASES_UNKNOWN "Onbekende alias %s."
#define MSG_ALIASES_BROKEN_FILE "Beschadigd aliassen bestand"
#define MSG_ALIASES_LIST "De volgende aliassen zijn beschikbaar :"
#define MSG_PERUSER_MAX "Ik kan niet meer dan %lu verbindingen van dezelfde gebruiker accepteren"
#define MSG_IPV6_OK "IPv6 verbindingen zijn ook welkom op deze server"
#define MSG_TLS_INFO "SSL/TLS: Enabled %s with %s, %d secret bits cipher"
#define MSG_TLS_WEAK "SSL/TLS: Cipher too weak"
#define MSG_TLS_NEEDED "Sorry, cleartext sessions are not accepted on this server.\n" \
    "Please reconnect using SSL/TLS security mechanisms."
#define MSG_ILLEGAL_CHARSET "Illegal charset"
#define MSG_TLS_NO_CTX "SSL/TLS: Context not found. Exiting."
#define MSG_PROT_OK "Data protection level set to \"%s\""
#define MSG_PROT_PRIVATE_NEEDED "Data connection cannot be opened with this PROT setting."
#define MSG_PROT_UNKNOWN_LEVEL "Protection level %s not understood. Fallback to \"%s\""
#define MSG_PROT_BEFORE_PBSZ "PROT must be preceded by a successful PBSZ command"
#define MSG_WARN_LDAP_USERPASS_EMPTY "Geen userPassword attribuut aangetroffen. Controleer de toegangsrechten."
#define MSG_LDAP_INVALID_AUTH_METHOD "Onjuiste LDAPAuthMethod in de configuratie. Moet 'bind' of 'password' zijn."