#include "wavetables.h"
// tablesize is 1024 + 1 wrap around
const float sintable[1025] = {
   0.0,
   0.0061418825059790854,
   0.0122835333212547,
   0.018424720763863454,
   0.024565213169321802,
   0.030704778899365143,
   0.036843186350685925,
   0.04298020396367041,
   0.04911560023113388,
   0.055249143707053715,
   0.061380603015300315,
   0.0675097468583653,
   0.07363634402608675,
   0.07976016340437117,
   0.08588097398391183,
   0.09199854486890317,
   0.09811264528575084,
   0.10422304459177724,
   0.11032951228392207,
   0.11643181800743756,
   0.12252973156457825,
   0.1286230229232847,
   0.13471146222586097,
   0.14079481979764566,
   0.14687286615567585,
   0.15294537201734398,
   0.159012108309047,
   0.16507284617482784,
   0.17112735698500853,
   0.1771754123448147,
   0.18321678410299155,
   0.1892512443604102,
   0.19527856547866485,
   0.20129852008866006,
   0.2073108810991877,
   0.21331542170549356,
   0.21931191539783312,
   0.2253001359700163,
   0.23127985752794047,
   0.237250854498112,
   0.2432129016361556,
   0.24916577403531112,
   0.2551092471349177,
   0.26104309672888526,
   0.26696709897415166,
   0.27288103039912714,
   0.27878466791212436,
   0.2846777888097739,
   0.29056017078542545,
   0.29643159193753393,
   0.3022918307780302,
   0.30814066624067626,
   0.31397787768940477,
   0.3198032449266417,
   0.32561654820161334,
   0.3314175682186357,
   0.3372060861453871,
   0.342981883621163,
   0.3487447427651136,
   0.3544944461844626,
   0.3602307769827083,
   0.36595351876780524,
   0.3716624556603275,
   0.37735737230161204,
   0.38303805386188283,
   0.38870428604835505,
   0.39435585511331855,
   0.3999925478622013,
   0.4056141516616117,
   0.4112204544473596,
   0.4168112447324564,
   0.42238631161509255,
   0.4279454447865937,
   0.4334884345393541,
   0.43901507177474736,
   0.44452514801101445,
   0.4500184553911281,
   0.455494786690634,
   0.4609539353254676,
   0.46639569535974756,
   0.4718198615135439,
   0.47722622917062196,
   0.4826145943861612,
   0.48798475389444856,
   0.4933365051165462,
   0.49866964616793347,
   0.5039839758661226,
   0.5092792937382481,
   0.5145554000286288,
   0.5198120957063036,
   0.5250491824725396,
   0.530266462768312,
   0.5354637397817574,
   0.5406408174555976,
   0.5457975004945353,
   0.5509335943726221,
   0.5560489053405959,
   0.5611432404331896,
   0.5662164074764112,
   0.5712682150947923,
   0.5762984727186076,
   0.5813069905910642,
   0.5862935797754594,
   0.5912580521623079,
   0.5962002204764384,
   0.6011198982840574,
   0.606016899999783,
   0.6108910408936447,
   0.615742137098053,
   0.6205700056147349,
   0.6253744643216368,
   0.6301553319797957,
   0.6349124282401752,
   0.6396455736504696,
   0.6443545896618722,
   0.649039298635812,
   0.6536995238506542,
   0.6583350895083666,
   0.6629458207411514,
   0.6675315436180417,
   0.6720920851514627,
   0.6766272733037573,
   0.6811369369936762,
   0.6856209061028309,
   0.6900790114821119,
   0.6945110849580691,
   0.6989169593392558,
   0.7032964684225358,
   0.707649446999353,
   0.7119757308619635,
   0.7162751568096305,
   0.7205475626547801,
   0.72479278722912,
   0.7290106703897187,
   0.7332010530250472,
   0.7373637770609812,
   0.7414986854667632,
   0.7456056222609273,
   0.7496844325171829,
   0.7537349623702585,
   0.7577570590217068,
   0.7617505707456682,
   0.7657153468945948,
   0.7696512379049323,
   0.7735580953027633,
   0.7774357717094076,
   0.7812841208469813,
   0.7851029975439158,
   0.7888922577404331,
   0.7926517584939814,
   0.796381357984626,
   0.8000809155204002,
   0.8037502915426121,
   0.8073893476311095,
   0.8109979465095011,
   0.8145759520503356,
   0.8181232292802364,
   0.8216396443849932,
   0.8251250647146099,
   0.8285793587883091,
   0.8320023962994914,
   0.8353940481206507,
   0.8387541863082459,
   0.8420826841075276,
   0.8453794159573185,
   0.8486442574947509,
   0.8518770855599579,
   0.8550777782007192,
   0.8582462146770613,
   0.861382275465813,
   0.8644858422651132,
   0.867556797998874,
   0.8705950268211977,
   0.8736004141207461,
   0.876572846525064,
   0.8795122119048566,
   0.8824183993782191,
   0.8852912993148189,
   0.8881308033400321,
   0.8909368043390312,
   0.893709196460826,
   0.8964478751222563,
   0.8991527370119378,
   0.9018236800941583,
   0.9044606036127277,
   0.9070634080947786,
   0.9096319953545183,
   0.9121662684969333,
   0.914666131921444,
   0.9171314913255112,
   0.9195622537081937,
   0.921958327373656,
   0.9243196219346282,
   0.926646048315815,
   0.928937518757256,
   0.9311939468176365,
   0.9334152473775483,
   0.9356013366427004,
   0.9377521321470804,
   0.9398675527560649,
   0.9419475186694806,
   0.9439919514246141,
   0.9460007738991725,
   0.9479739103141916,
   0.9499112862368959,
   0.951812828583505,
   0.9536784656219918,
   0.9555081269747874,
   0.9573017436214368,
   0.9590592479012021,
   0.9607805735156149,
   0.9624656555309772,
   0.9641144303808117,
   0.9657268358682582,
   0.9673028111684215,
   0.9688422968306648,
   0.9703452347808526,
   0.9718115683235417,
   0.9732412421441198,
   0.9746342023108919,
   0.9759903962771153,
   0.9773097728829812,
   0.9785922823575449,
   0.9798378763206036,
   0.9810465077845208,
   0.9822181311559995,
   0.9833527022378016,
   0.9844501782304154,
   0.98551051773367,
   0.9865336807482974,
   0.9875196286774409,
   0.9884683243281114,
   0.9893797319125902,
   0.9902538170497794,
   0.9910905467664985,
   0.9918898894987285,
   0.9926518150928021,
   0.9933762948065421,
   0.9940633013103448,
   0.9947128086882113,
   0.9953247924387252,
   0.9958992294759764,
   0.996436098130433,
   0.9969353781497574,
   0.9973970506995714,
   0.997821098364166,
   0.9982075051471587,
   0.998556256472097,
   0.998867339183008,
   0.9991407415448946,
   0.9993764532441789,
   0.9995744653890902,
   0.9997347705100013,
   0.9998573625597098,
   0.9999422369136666,
   0.9999893903701498,
   0.999998821150386,
   0.9999705288986172,
   0.9999045146821139,
   0.9998007809911359,
   0.9996593317388369,
   0.9994801722611178,
   0.9992633093164256,
   0.9990087510854974,
   0.9987165071710528,
   0.9983865885974316,
   0.9980190078101772,
   0.9976137786755678,
   0.997170916480093,
   0.9966904379298773,
   0.9961723611500499,
   0.9956167056840606,
   0.9950234924929431,
   0.994392743954524,
   0.9937244838625787,
   0.9930187374259336,
   0.9922755312675157,
   0.9914948934233476,
   0.9906768533414904,
   0.9898214418809328,
   0.9889286913104265,
   0.9879986353072698,
   0.9870313089560363,
   0.986026748747252,
   0.9849849925760185,
   0.9839060797405833,
   0.982790050940858,
   0.9816369482768823,
   0.9804468152472362,
   0.9792196967473991,
   0.9779556390680557,
   0.9766546898933508,
   0.9753168982990894,
   0.9739423147508863,
   0.9725309911022617,
   0.9710829805926857,
   0.9695983378455693,
   0.9680771188662043,
   0.9665193810396508,
   0.964925183128572,
   0.9632945852710174,
   0.9616276489781548,
   0.9599244371319494,
   0.9581850139827921,
   0.9564094451470752,
   0.9545977976047176,
   0.9527501396966384,
   0.9508665411221777,
   0.9489470729364691,
   0.9469918075477574,
   0.9450008187146686,
   0.9429741815434264,
   0.9409119724850202,
   0.9388142693323197,
   0.9366811512171416,
   0.9345126986072634,
   0.932308993303389,
   0.9300701184360619,
   0.9277961584625302,
   0.9254871991635596,
   0.9231433276401985,
   0.9207646323104914,
   0.9183512029061446,
   0.9159031304691395,
   0.9134205073483003,
   0.9109034271958085,
   0.9083519849636714,
   0.9057662769001396,
   0.9031464005460758,
   0.9004924547312763,
   0.8978045395707416,
   0.8950827564609015,
   0.892327208075788,
   0.8895379983631637,
   0.8867152325405996,
   0.8838590170915072,
   0.88096945976112,
   0.8780466695524302,
   0.8750907567220761,
   0.872101832776184,
   0.8690800104661602,
   0.8660254037844387,
   0.8629381279601812,
   0.8598182994549293,
   0.8566660359582128,
   0.8534814563831084,
   0.8502646808617549,
   0.8470158307408212,
   0.8437350285769293,
   0.8404223981320298,
   0.8370780643687344,
   0.8337021534456012,
   0.8302947927123764,
   0.8268561107051895,
   0.8233862371417048,
   0.8198853029162276,
   0.8163534400947683,
   0.8127907819100577,
   0.8091974627565232,
   0.8055736181852173,
   0.801919384898707,
   0.7982349007459133,
   0.7945203047169139,
   0.7907757369376985,
   0.7870013386648845,
   0.7831972522803862,
   0.7793636212860452,
   0.7755005902982163,
   0.7716083050423137,
   0.7676869123473112,
   0.7637365601402055,
   0.7597573974404348,
   0.7557495743542583,
   0.7517132420690926,
   0.7476485528478092,
   0.74355566002299,
   0.7394347179911447,
   0.7352858822068845,
   0.731109309177059,
   0.7269051564548515,
   0.7226735826338374,
   0.7184147473419993,
   0.7141288112357064,
   0.7098159359936541,
   0.7054762843107656,
   0.7011100198920528,
   0.6967173074464422,
   0.6922983126805615,
   0.6878532022924878,
   0.6833821439654609,
   0.6788853063615556,
   0.6743628591153208,
   0.6698149728273793,
   0.6652418190579943,
   0.6606435703205946,
   0.6560204000752692,
   0.6513724827222223,
   0.6466999935951963,
   0.6420031089548556,
   0.6372820059821387,
   0.6325368627715737,
   0.6277678583245616,
   0.6229751725426214,
   0.6181589862206055,
   0.6133194810398783,
   0.6084568395614648,
   0.6035712452191607,
   0.5986628823126153,
   0.5937319360003773,
   0.5887785922929124,
   0.5838030380455836,
   0.5788054609516045,
   0.5737860495349573,
   0.5687449931432838,
   0.5636824819407391,
   0.5585987069008207,
   0.5534938597991627,
   0.5483681332063035,
   0.543221720480419,
   0.5380548157600299,
   0.5328676139566779,
   0.527660310747574,
   0.5224331025682151,
   0.5171861866049748,
   0.5119197607876653,
   0.5066340237820709,
   0.5013291749824526,
   0.49600541450402735,
   0.4906629431754188,
   0.48530196253108104,
   0.47992267480369777,
   0.4745252829165508,
   0.4691099904758671,
   0.4636770017631372,
   0.4582265217274105,
   0.4527587559775619,
   0.4472739107745369,
   0.44177219302357007,
   0.43625381026638177,
   0.43071897067334636,
   0.425167883035641,
   0.41960075675736885,
   0.41401780184766107,
   0.4084192289127522,
   0.4028052491480372,
   0.39717607433010377,
   0.3915319168087448,
   0.38587298949894566,
   0.38019950587285406,
   0.37451167995172635,
   0.36880972629785547,
   0.36309386000647476,
   0.35736429669764547,
   0.35162125250812204,
   0.3458649440832005,
   0.34009558856854333,
   0.3343134036019898,
   0.3285186073053452,
   0.32271141827615424,
   0.3168920555794523,
   0.31106073873950346,
   0.3052176877315184,
   0.29936312297335804,
   0.2934972653172159,
   0.2876203360412891,
   0.28173255684142967,
   0.2758341498227834,
   0.26992533749140907,
   0.26400634274588597,
   0.25807738886890524,
   0.25213869951884815,
   0.24619049872134688,
   0.24023301086083496,
   0.2342664606720825,
   0.2282910732317182,
   0.2223070739497403,
   0.21631468856101088,
   0.2103141431167422,
   0.20430566397596858,
   0.19828947779700898,
   0.19226581152891425,
   0.18623489240290775,
   0.18019694792381238,
   0.1741522058614704,
   0.1681008942421484,
   0.16204324133993744,
   0.15597947566814074,
   0.14990982597065533,
   0.1438345212133405,
   0.1377537905753822,
   0.1316678634406469,
   0.12557696938903015,
   0.11948133818779369,
   0.11338119978289944,
   0.10727678429033452,
   0.10116832198743228,
   0.09505604330418288,
   0.08894017881454251,
   0.08282095922773458,
   0.07669861537954843,
   0.0705733782236288,
   0.06444547882276536,
   0.05831514834017538,
   0.052182618030785353,
   0.0460481192325047,
   0.03991188335750071,
   0.03377414188346813,
   0.02763512634489885,
   0.021495068324345064,
   0.01535419944368505,
   0.009212751355384765,
   0.003070955733761002,
   -0.003070955733760757,
   -0.00921275135538452,
   -0.01535419944368436,
   -0.02149506832434482,
   -0.027635126344898602,
   -0.03377414188346788,
   -0.039911883357500025,
   -0.04604811923250445,
   -0.05218261803078511,
   -0.05831514834017513,
   -0.06444547882276468,
   -0.07057337822362857,
   -0.0766986153795482,
   -0.08282095922773434,
   -0.08894017881454182,
   -0.09505604330418263,
   -0.10116832198743204,
   -0.10727678429033427,
   -0.11338119978289875,
   -0.11948133818779345,
   -0.12557696938902993,
   -0.13166786344064665,
   -0.13775379057538154,
   -0.14383452121334026,
   -0.14990982597065508,
   -0.1559794756681405,
   -0.16204324133993678,
   -0.1681008942421482,
   -0.17415220586147015,
   -0.18019694792381213,
   -0.18623489240290708,
   -0.19226581152891403,
   -0.19828947779700873,
   -0.20430566397596836,
   -0.21031414311674154,
   -0.21631468856101063,
   -0.22230707394974006,
   -0.228291073231718,
   -0.23426646067208184,
   -0.2402330108608347,
   -0.24619049872134666,
   -0.25213869951884793,
   -0.25807738886890497,
   -0.2640063427458853,
   -0.2699253374914088,
   -0.2758341498227832,
   -0.2817325568414294,
   -0.28762033604128845,
   -0.2934972653172157,
   -0.29936312297335776,
   -0.3052176877315182,
   -0.3110607387395028,
   -0.31689205557945205,
   -0.322711418276154,
   -0.328518607305345,
   -0.33431340360198913,
   -0.3400955885685431,
   -0.3458649440832003,
   -0.3516212525081218,
   -0.3573642966976448,
   -0.36309386000647453,
   -0.36880972629785524,
   -0.37451167995172613,
   -0.38019950587285345,
   -0.38587298949894544,
   -0.39153191680874455,
   -0.39717607433010355,
   -0.4028052491480366,
   -0.40841922891275195,
   -0.41401780184766085,
   -0.41960075675736863,
   -0.4251678830356404,
   -0.43071897067334614,
   -0.43625381026638155,
   -0.44177219302356985,
   -0.44727391077453627,
   -0.45275875597756166,
   -0.4582265217274103,
   -0.463677001763137,
   -0.4691099904758665,
   -0.47452528291655055,
   -0.47992267480369755,
   -0.4853019625310808,
   -0.4906629431754182,
   -0.49600541450402713,
   -0.5013291749824523,
   -0.5066340237820707,
   -0.5119197607876651,
   -0.5171861866049743,
   -0.5224331025682148,
   -0.5276603107475738,
   -0.5328676139566777,
   -0.5380548157600293,
   -0.5432217204804187,
   -0.5483681332063033,
   -0.5534938597991624,
   -0.55859870690082,
   -0.5636824819407389,
   -0.5687449931432835,
   -0.5737860495349572,
   -0.5788054609516039,
   -0.5838030380455834,
   -0.5887785922929122,
   -0.5937319360003771,
   -0.5986628823126148,
   -0.6035712452191605,
   -0.6084568395614646,
   -0.6133194810398781,
   -0.618158986220605,
   -0.6229751725426212,
   -0.6277678583245614,
   -0.6325368627715735,
   -0.6372820059821381,
   -0.6420031089548554,
   -0.646699993595196,
   -0.651372482722222,
   -0.6560204000752686,
   -0.6606435703205945,
   -0.6652418190579942,
   -0.6698149728273792,
   -0.6743628591153202,
   -0.6788853063615554,
   -0.6833821439654607,
   -0.6878532022924877,
   -0.6922983126805611,
   -0.696717307446442,
   -0.7011100198920526,
   -0.7054762843107654,
   -0.709815935993654,
   -0.7141288112357059,
   -0.7184147473419991,
   -0.7226735826338372,
   -0.7269051564548513,
   -0.7311093091770585,
   -0.7352858822068844,
   -0.7394347179911446,
   -0.7435556600229899,
   -0.7476485528478087,
   -0.7517132420690924,
   -0.7557495743542582,
   -0.7597573974404349,
   -0.7637365601402054,
   -0.7676869123473111,
   -0.7716083050423135,
   -0.7755005902982162,
   -0.7793636212860447,
   -0.7831972522803857,
   -0.7870013386648841,
   -0.7907757369376986,
   -0.7945203047169137,
   -0.7982349007459132,
   -0.8019193848987068,
   -0.8055736181852172,
   -0.8091974627565227,
   -0.8127907819100574,
   -0.816353440094768,
   -0.8198853029162277,
   -0.8233862371417046,
   -0.8268561107051894,
   -0.8302947927123763,
   -0.833702153445601,
   -0.837078064368734,
   -0.8404223981320295,
   -0.843735028576929,
   -0.8470158307408213,
   -0.8502646808617548,
   -0.8534814563831082,
   -0.8566660359582127,
   -0.8598182994549292,
   -0.8629381279601809,
   -0.8660254037844385,
   -0.8690800104661598,
   -0.8721018327761837,
   -0.8750907567220763,
   -0.87804666955243,
   -0.8809694597611198,
   -0.883859017091507,
   -0.8867152325405995,
   -0.8895379983631634,
   -0.8923272080757877,
   -0.8950827564609012,
   -0.8978045395707417,
   -0.9004924547312761,
   -0.9031464005460756,
   -0.9057662769001394,
   -0.9083519849636713,
   -0.9109034271958083,
   -0.9134205073482999,
   -0.9159031304691392,
   -0.9183512029061445,
   -0.9207646323104914,
   -0.9231433276401984,
   -0.9254871991635595,
   -0.92779615846253,
   -0.9300701184360618,
   -0.9323089933033888,
   -0.9345126986072632,
   -0.9366811512171416,
   -0.9388142693323197,
   -0.9409119724850201,
   -0.9429741815434264,
   -0.9450008187146683,
   -0.9469918075477572,
   -0.9489470729364688,
   -0.9508665411221775,
   -0.9527501396966384,
   -0.9545977976047176,
   -0.956409445147075,
   -0.958185013982792,
   -0.9599244371319493,
   -0.9616276489781547,
   -0.9632945852710172,
   -0.9649251831285718,
   -0.9665193810396508,
   -0.9680771188662043,
   -0.9695983378455693,
   -0.9710829805926856,
   -0.9725309911022617,
   -0.9739423147508862,
   -0.9753168982990893,
   -0.9766546898933506,
   -0.9779556390680555,
   -0.9792196967473991,
   -0.9804468152472362,
   -0.9816369482768823,
   -0.9827900509408579,
   -0.9839060797405832,
   -0.9849849925760183,
   -0.9860267487472519,
   -0.9870313089560362,
   -0.9879986353072698,
   -0.9889286913104265,
   -0.9898214418809327,
   -0.9906768533414904,
   -0.9914948934233475,
   -0.9922755312675157,
   -0.9930187374259336,
   -0.9937244838625786,
   -0.994392743954524,
   -0.9950234924929431,
   -0.9956167056840606,
   -0.9961723611500499,
   -0.9966904379298773,
   -0.997170916480093,
   -0.9976137786755678,
   -0.9980190078101772,
   -0.9983865885974316,
   -0.9987165071710528,
   -0.9990087510854974,
   -0.9992633093164255,
   -0.9994801722611178,
   -0.9996593317388369,
   -0.9998007809911359,
   -0.9999045146821139,
   -0.9999705288986172,
   -0.999998821150386,
   -0.9999893903701498,
   -0.9999422369136666,
   -0.9998573625597098,
   -0.9997347705100013,
   -0.9995744653890902,
   -0.9993764532441789,
   -0.9991407415448947,
   -0.998867339183008,
   -0.998556256472097,
   -0.9982075051471587,
   -0.997821098364166,
   -0.9973970506995714,
   -0.9969353781497574,
   -0.996436098130433,
   -0.9958992294759765,
   -0.9953247924387252,
   -0.9947128086882113,
   -0.9940633013103448,
   -0.9933762948065422,
   -0.9926518150928022,
   -0.9918898894987285,
   -0.9910905467664987,
   -0.9902538170497795,
   -0.9893797319125902,
   -0.9884683243281114,
   -0.9875196286774409,
   -0.9865336807482975,
   -0.98551051773367,
   -0.9844501782304154,
   -0.9833527022378017,
   -0.9822181311559997,
   -0.9810465077845208,
   -0.9798378763206036,
   -0.9785922823575449,
   -0.9773097728829813,
   -0.9759903962771154,
   -0.974634202310892,
   -0.9732412421441199,
   -0.9718115683235419,
   -0.9703452347808526,
   -0.9688422968306648,
   -0.9673028111684215,
   -0.9657268358682582,
   -0.9641144303808117,
   -0.9624656555309774,
   -0.960780573515615,
   -0.9590592479012022,
   -0.957301743621437,
   -0.9555081269747875,
   -0.9536784656219918,
   -0.9518128285835051,
   -0.949911286236896,
   -0.9479739103141918,
   -0.9460007738991726,
   -0.9439919514246143,
   -0.9419475186694809,
   -0.9398675527560649,
   -0.9377521321470804,
   -0.9356013366427005,
   -0.9334152473775484,
   -0.9311939468176367,
   -0.9289375187572562,
   -0.9266460483158152,
   -0.9243196219346286,
   -0.921958327373656,
   -0.9195622537081937,
   -0.9171314913255113,
   -0.9146661319214441,
   -0.9121662684969335,
   -0.9096319953545186,
   -0.9070634080947789,
   -0.9044606036127281,
   -0.9018236800941583,
   -0.8991527370119378,
   -0.8964478751222564,
   -0.8937091964608261,
   -0.8909368043390314,
   -0.8881308033400324,
   -0.8852912993148192,
   -0.8824183993782194,
   -0.8795122119048566,
   -0.8765728465250641,
   -0.8736004141207462,
   -0.8705950268211979,
   -0.8675567979988742,
   -0.8644858422651134,
   -0.8613822754658133,
   -0.8582462146770616,
   -0.8550777782007195,
   -0.8518770855599579,
   -0.8486442574947509,
   -0.8453794159573186,
   -0.8420826841075277,
   -0.8387541863082462,
   -0.8353940481206509,
   -0.8320023962994917,
   -0.8285793587883096,
   -0.8251250647146099,
   -0.8216396443849933,
   -0.8181232292802365,
   -0.8145759520503358,
   -0.8109979465095014,
   -0.8073893476311098,
   -0.8037502915426125,
   -0.8000809155204006,
   -0.796381357984626,
   -0.7926517584939814,
   -0.7888922577404333,
   -0.7851029975439159,
   -0.7812841208469815,
   -0.7774357717094079,
   -0.7735580953027638,
   -0.7696512379049327,
   -0.7657153468945946,
   -0.7617505707456682,
   -0.7577570590217069,
   -0.7537349623702586,
   -0.7496844325171831,
   -0.7456056222609276,
   -0.7414986854667636,
   -0.7373637770609817,
   -0.7332010530250472,
   -0.7290106703897187,
   -0.72479278722912,
   -0.7205475626547803,
   -0.7162751568096307,
   -0.7119757308619639,
   -0.7076494469993533,
   -0.7032964684225362,
   -0.6989169593392565,
   -0.6945110849580691,
   -0.690079011482112,
   -0.6856209061028311,
   -0.6811369369936764,
   -0.6766272733037577,
   -0.6720920851514631,
   -0.6675315436180422,
   -0.662945820741152,
   -0.6583350895083666,
   -0.6536995238506543,
   -0.6490392986358122,
   -0.6443545896618724,
   -0.6396455736504699,
   -0.6349124282401758,
   -0.6301553319797962,
   -0.6253744643216375,
   -0.6205700056147349,
   -0.6157421370980531,
   -0.6108910408936449,
   -0.6060168999997833,
   -0.6011198982840578,
   -0.5962002204764387,
   -0.5912580521623084,
   -0.5862935797754599,
   -0.5813069905910642,
   -0.5762984727186076,
   -0.5712682150947924,
   -0.5662164074764114,
   -0.56114324043319,
   -0.5560489053405963,
   -0.5509335943726227,
   -0.5457975004945359,
   -0.5406408174555974,
   -0.5354637397817575,
   -0.5302664627683121,
   -0.5250491824725398,
   -0.5198120957063039,
   -0.5145554000286292,
   -0.5092792937382485,
   -0.5039839758661232,
   -0.4986696461679334,
   -0.4933365051165462,
   -0.48798475389444873,
   -0.48261459438616144,
   -0.4772262291706223,
   -0.47181986151354427,
   -0.46639569535974806,
   -0.4609539353254682,
   -0.4554947866906347,
   -0.4500184553911281,
   -0.44452514801101456,
   -0.4390150717747475,
   -0.4334884345393544,
   -0.4279454447865941,
   -0.422386311615093,
   -0.41681124473245695,
   -0.4112204544473603,
   -0.40561415166161163,
   -0.3999925478622014,
   -0.3943558551133187,
   -0.38870428604835533,
   -0.3830380538618832,
   -0.3773573723016125,
   -0.3716624556603281,
   -0.36595351876780596,
   -0.3602307769827083,
   -0.3544944461844627,
   -0.34874474276511375,
   -0.3429818836211633,
   -0.3372060861453875,
   -0.3314175682186362,
   -0.32561654820161395,
   -0.31980324492664236,
   -0.31397787768940466,
   -0.3081406662406763,
   -0.30229183077803035,
   -0.2964315919375342,
   -0.29056017078542584,
   -0.28467778880977435,
   -0.2787846679121249,
   -0.2728810303991278,
   -0.26696709897415155,
   -0.2610430967288853,
   -0.2551092471349179,
   -0.24916577403531134,
   -0.24321290163615597,
   -0.23725085449811248,
   -0.23127985752794103,
   -0.22530013597001694,
   -0.21931191539783387,
   -0.21331542170549353,
   -0.2073108810991878,
   -0.20129852008866028,
   -0.19527856547866518,
   -0.18925124436041063,
   -0.18321678410299208,
   -0.17717541234481535,
   -0.17112735698500928,
   -0.16507284617482784,
   -0.15901210830904708,
   -0.15294537201734418,
   -0.14687286615567616,
   -0.14079481979764608,
   -0.1347114622258615,
   -0.1286230229232853,
   -0.122529731564579,
   -0.11643181800743753,
   -0.11032951228392213,
   -0.10422304459177742,
   -0.09811264528575113,
   -0.09199854486890356,
   -0.08588097398391234,
   -0.07976016340437178,
   -0.07363634402608747,
   -0.06750974685836525,
   -0.061380603015300364,
   -0.055249143707053874,
   -0.04911560023113414,
   -0.04298020396367079,
   -0.036843186350686404,
   -0.030704778899365737,
   -0.024565213169322503,
   -0.01842472076386337,
   -0.012283533321254727,
   -0.0061418825059792225,
   -2.4492935982947064e-16,
   0.0
};
