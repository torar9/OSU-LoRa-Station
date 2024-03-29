PGDMP     %                    z           ou_stations #   14.3 (Ubuntu 14.3-0ubuntu0.22.04.1)    14.1                0    0    ENCODING    ENCODING        SET client_encoding = 'UTF8';
                      false                       0    0 
   STDSTRINGS 
   STDSTRINGS     (   SET standard_conforming_strings = 'on';
                      false            	           0    0 
   SEARCHPATH 
   SEARCHPATH     8   SELECT pg_catalog.set_config('search_path', '', false);
                      false            
           1262    16384    ou_stations    DATABASE     \   CREATE DATABASE ou_stations WITH TEMPLATE = template0 ENCODING = 'UTF8' LOCALE = 'C.UTF-8';
    DROP DATABASE ou_stations;
                postgres    false                        2615    16385    ttn    SCHEMA        CREATE SCHEMA ttn;
    DROP SCHEMA ttn;
                postgres    false            �            1259    16386    application    TABLE     U   CREATE TABLE ttn.application (
    application_id character varying(255) NOT NULL
);
    DROP TABLE ttn.application;
       ttn         heap    postgres    false    6            �            1259    16389    device    TABLE        CREATE TABLE ttn.device (
    device_id character varying(255) NOT NULL,
    application_id character varying(255) NOT NULL
);
    DROP TABLE ttn.device;
       ttn         heap    postgres    false    6            �            1259    16460    uplink    TABLE     �  CREATE TABLE ttn.uplink (
    uplink_id bigint NOT NULL,
    device_id character varying(255) NOT NULL,
    temperature numeric NOT NULL,
    humidity numeric NOT NULL,
    nc0p5 numeric NOT NULL,
    nc1p0 numeric NOT NULL,
    nc10p0 numeric NOT NULL,
    nc2p5 numeric NOT NULL,
    nc4p0 numeric NOT NULL,
    pm1p0 numeric NOT NULL,
    pm10p0 numeric NOT NULL,
    pm2p5 numeric NOT NULL,
    pm4p0 numeric NOT NULL,
    typical_value numeric NOT NULL,
    simulated boolean DEFAULT false NOT NULL,
    gateway_id character varying(255) NOT NULL,
    rssi numeric,
    channel_rssi numeric,
    snr numeric,
    received_at timestamp with time zone NOT NULL,
    saved_at timestamp with time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
    raw_json json
);
    DROP TABLE ttn.uplink;
       ttn         heap    postgres    false    6            �            1259    16459    uplink_measurement_id_seq    SEQUENCE     �   ALTER TABLE ttn.uplink ALTER COLUMN uplink_id ADD GENERATED ALWAYS AS IDENTITY (
    SEQUENCE NAME ttn.uplink_measurement_id_seq
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1
);
            ttn          postgres    false    6    213                      0    16386    application 
   TABLE DATA           2   COPY ttn.application (application_id) FROM stdin;
    ttn          postgres    false    210   �                 0    16389    device 
   TABLE DATA           8   COPY ttn.device (device_id, application_id) FROM stdin;
    ttn          postgres    false    211   �                 0    16460    uplink 
   TABLE DATA           �   COPY ttn.uplink (uplink_id, device_id, temperature, humidity, nc0p5, nc1p0, nc10p0, nc2p5, nc4p0, pm1p0, pm10p0, pm2p5, pm4p0, typical_value, simulated, gateway_id, rssi, channel_rssi, snr, received_at, saved_at, raw_json) FROM stdin;
    ttn          postgres    false    213   �                  0    0    uplink_measurement_id_seq    SEQUENCE SET     F   SELECT pg_catalog.setval('ttn.uplink_measurement_id_seq', 102, true);
          ttn          postgres    false    212            o           2606    16404    application application_pkey 
   CONSTRAINT     c   ALTER TABLE ONLY ttn.application
    ADD CONSTRAINT application_pkey PRIMARY KEY (application_id);
 C   ALTER TABLE ONLY ttn.application DROP CONSTRAINT application_pkey;
       ttn            postgres    false    210            q           2606    16406    device device_pkey 
   CONSTRAINT     T   ALTER TABLE ONLY ttn.device
    ADD CONSTRAINT device_pkey PRIMARY KEY (device_id);
 9   ALTER TABLE ONLY ttn.device DROP CONSTRAINT device_pkey;
       ttn            postgres    false    211            s           2606    16468    uplink uplink_pkey 
   CONSTRAINT     T   ALTER TABLE ONLY ttn.uplink
    ADD CONSTRAINT uplink_pkey PRIMARY KEY (uplink_id);
 9   ALTER TABLE ONLY ttn.uplink DROP CONSTRAINT uplink_pkey;
       ttn            postgres    false    213            t           2606    16409 %   device applciation_device_foreign_key    FK CONSTRAINT     �   ALTER TABLE ONLY ttn.device
    ADD CONSTRAINT applciation_device_foreign_key FOREIGN KEY (application_id) REFERENCES ttn.application(application_id) NOT VALID;
 L   ALTER TABLE ONLY ttn.device DROP CONSTRAINT applciation_device_foreign_key;
       ttn          postgres    false    210    211    3183            u           2606    16469     uplink device_uplink_foreign_key    FK CONSTRAINT     �   ALTER TABLE ONLY ttn.uplink
    ADD CONSTRAINT device_uplink_foreign_key FOREIGN KEY (device_id) REFERENCES ttn.device(device_id);
 G   ALTER TABLE ONLY ttn.uplink DROP CONSTRAINT device_uplink_foreign_key;
       ttn          postgres    false    213    3185    211                  x������ � �            x������ � �            x������ � �     